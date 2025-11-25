#!/usr/bin/env python3
import time, math, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue
from rcl_interfaces.srv import SetParameters
from unitree_api.msg import Request as UnitreeRequest

class HandoverCoordinator(Node):
    def __init__(self):
        super().__init__("handover_coordinator")

        # Topics
        self.sub_ready = self.create_subscription(Bool, "/approach/ready", self.on_ready, 10)
        self.sub_hold  = self.create_subscription(Bool, "/perception/handover_ready", self.on_hold, 10)
        self.sub_armst = self.create_subscription(String, "/arm/state", self.on_arm_state, 10)

        self.pub_arrived = self.create_publisher(Bool, "/approach/arrived", 10)
        self.pub_cmdvel  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_prog    = self.create_publisher(UnitreeRequest, "/api/programming_actuator/request", 10)

        # Params (tweak without recompiling)
        self.declare_parameter("arm_node_name", "arm_impedance")
        self.declare_parameter("pregrasp_offset_pre", [-0.08, 0.0, +0.04])   # m (x towards robot)
        self.declare_parameter("pregrasp_offset_grasp", [-0.01, 0.0, +0.02]) # m
        self.declare_parameter("pregrasp_offset_retract", [-0.25, 0.0, +0.06])
        self.declare_parameter("arm_settle_err_m", 0.02)
        self.declare_parameter("arm_settle_yaw_rad", 0.15)
        self.declare_parameter("arm_settle_win", 8)  # consecutive ticks
        self.declare_parameter("gripper_open_api", 3001)   # TODO: set per Unitree D1 firmware
        self.declare_parameter("gripper_close_api", 3002)  # TODO: set per Unitree D1 firmware
        self.declare_parameter("gripper_param_open", "{}")
        self.declare_parameter("gripper_param_close", "{}")
        self.declare_parameter("backaway_vx", -0.15)
        self.declare_parameter("backaway_time_s", 0.8)

        self.state = "WAIT"
        self.approach_ready = False
        self.hold_ready = False
        self.arm_err = (999.0, 999.0)  # (pos_norm, yaw_err)
        self.settle_count = 0

        self.t_last = self.get_clock().now().nanoseconds * 1e-9
        self.timer = self.create_timer(0.05, self.tick)

        # Param client for arm_impedance pregrasp_offset
        self.arm_node = self.get_parameter("arm_node_name").get_parameter_value().string_value
        self.param_cli = self.create_client(SetParameters, f"/{self.arm_node}/set_parameters")

        self.get_logger().info("handover_coordinator ready.")

    # ---------- subs ----------
    def on_ready(self, msg: Bool): self.approach_ready = bool(msg.data)
    def on_hold(self, msg: Bool):  self.hold_ready = bool(msg.data)

    def on_arm_state(self, msg: String):
        # Parsing: "impedance_step | ex=[dx,dy,dz] eyaw=..."; robust default
        txt = msg.data
        try:
            parts = txt.split("ex=")[1]
            vec = parts.split("]")[0].strip("[]").split(",")
            dx,dy,dz = [float(v) for v in vec]
            yaw = float(txt.split("eyaw=")[1].split()[0])
            pos_norm = (dx*dx + dy*dy + dz*dz)**0.5
            self.arm_err = (pos_norm, abs(yaw))
        except Exception:
            pass

    # ---------- helpers ----------
    def set_pregrasp_offset(self, arr):
        if not self.param_cli.service_is_ready():
            self.param_cli.wait_for_service(timeout_sec=1.0)
        req = SetParameters.Request()
        pv = ParameterValue()
        pv.type = pv.TYPE_DOUBLE_ARRAY
        pv.double_array_value = [float(v) for v in arr]
        pm = ParamMsg(name="pregrasp_offset", value=pv)
        req.parameters = [pm]
        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)

    def publish_prog(self, api_id: int, param: str = "{}"):
        m = UnitreeRequest()
        m.header.identity.api_id = int(api_id)
        m.parameter = param
        self.pub_prog.publish(m)

    def cmdvel(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist(); t.linear.x=vx; t.linear.y=vy; t.angular.z=wz
        self.pub_cmdvel.publish(t)

    def arm_settled(self):
        pos_n, yaw = self.arm_err
        pos_ok = pos_n <= float(self.get_parameter("arm_settle_err_m").value)
        yaw_ok = yaw   <= float(self.get_parameter("arm_settle_yaw_rad").value)
        if pos_ok and yaw_ok: self.settle_count += 1
        else: self.settle_count = 0
        return self.settle_count >= int(self.get_parameter("arm_settle_win").value)

    # ---------- main loop ----------
    def tick(self):
        if self.state == "WAIT":
            if self.approach_ready and self.hold_ready:
                # Open gripper
                self.publish_prog(self.get_parameter("gripper_open_api").value,
                                  self.get_parameter("gripper_param_open").value)
                # Unfreeze arm controller via /approach/arrived
                self.pub_arrived.publish(Bool(data=True))
                # Pre‑grasp safe offset
                self.set_pregrasp_offset(self.get_parameter("pregrasp_offset_pre").value)
                self.state = "PREGRASP"
                self.get_logger().info("→ PREGRASP")

        elif self.state == "PREGRASP":
            if self.arm_settled():
                # Closer approach to actual grasp
                self.set_pregrasp_offset(self.get_parameter("pregrasp_offset_grasp").value)
                self.state = "GRASP"
                self.get_logger().info("→ GRASP")

        elif self.state == "GRASP":
            if self.arm_settled():
                self.publish_prog(self.get_parameter("gripper_close_api").value,
                                  self.get_parameter("gripper_param_close").value)
                # Small lift + retract
                self.set_pregrasp_offset(self.get_parameter("pregrasp_offset_retract").value)
                self.state = "RETRACT"
                self.get_logger().info("→ RETRACT")

        elif self.state == "RETRACT":
            if self.arm_settled():
                # Short back‑away step (bridge will convert to Unitree 1008)
                vx = float(self.get_parameter("backaway_vx").value)
                T  = float(self.get_parameter("backaway_time_s").value)
                t0 = time.time()
                while time.time() - t0 < T:
                    self.cmdvel(vx=vx)
                    rclpy.spin_once(self, timeout_sec=0.01)
                self.cmdvel(0,0,0)
                # Stand (bridge sees zero and issues STAND)
                self.state = "DONE"
                self.get_logger().info("→ DONE")

        elif self.state == "DONE":
            pass

def main():
    rclpy.init()
    n = HandoverCoordinator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
