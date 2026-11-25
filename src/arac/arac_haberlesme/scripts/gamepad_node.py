#!/usr/bin/env python3
"""
ZBEU OVAT - Gamepad ROS2 node'u
--------------------------------
Logitech F710'u okur ve durumunu ROS'a yayinlar. SERI PORTA YAZMAZ!
Teensy'ye yazan TEK yer serial_comm_node'dur (tek-yazici mimarisi).

Yayin:  /gamepad/state  (std_msgs/Int32MultiArray)
        data = [steer(25-165), gaz, fren, geri, otonom, ACIL]

Kumanda haritasi:
    Sag stick yatay .. direksiyon
    Sol stick ILERI .. gaz
    RT ............... fren
    RB ............... vites toggle (ileri/geri)
    LB ............... OTONOM toggle (manuel <-> otonom)
    B  ............... (ACIL DURUM KALDIRILDI - buton okunmuyor)

Kurulum:  sudo apt install python3-pygame
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

try:
    import pygame
except ImportError:
    print("HATA: pygame yok ->  sudo apt install python3-pygame")
    sys.exit(1)

# Eksen indexleri PLATFORMA GORE degisir:
#   Windows (XInput)  -> sag stick yatay = 2
#   Linux (SDL2/xpad) -> eksen 2 = LT TETIGI (bosta -1!), sag stick = 3
AX_STEER_VARSAYILAN = 2 if sys.platform.startswith("win") else 3
AX_THR   = 1      # sol stick dikey: ileri = negatif
AX_RT    = 5      # RT: bosta -1, basili +1 (fren)
BTN_RB   = 5      # vites toggle
BTN_LB   = 4      # otonom toggle
BTN_B    = 1      # ACIL DURUM - ARTIK KULLANILMIYOR (bkz. tick())
DEADZONE = 0.3


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")

        self.declare_parameter("steer_axis", AX_STEER_VARSAYILAN)
        self.ax_steer = int(self.get_parameter("steer_axis").value)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error(
                "KUMANDA YOK! Dongle takili + gamepad acik + on anahtar 'X' modunda mi?")
            raise SystemExit(1)
        self.js = pygame.joystick.Joystick(0)
        self.js.init()

        pygame.event.pump()
        eksenler = "  ".join(f"{i}:{self.js.get_axis(i):+.2f}"
                             for i in range(self.js.get_numaxes()))
        self.get_logger().info(f"Kumanda: {self.js.get_name()}")
        self.get_logger().info(
            f"Direksiyon ekseni: {self.ax_steer} | bostaki eksenler: {eksenler}")
        if (self.ax_steer < self.js.get_numaxes()
                and abs(self.js.get_axis(self.ax_steer)) > 0.8):
            self.get_logger().warn(
                f"Eksen {self.ax_steer} bosta +-1 -> bu bir TETIK olabilir. "
                f"Duzelt: --ros-args -p steer_axis:=<N>")

        self.pub = self.create_publisher(Int32MultiArray, "/gamepad/state", 10)

        self.reverse = False
        self.otonom = False
        self.rb_prev = 0
        self.lb_prev = 0

        self.timer = self.create_timer(0.02, self.tick)   # 50 Hz
        self.get_logger().info("Gamepad node hazir -> /gamepad/state")

    def tick(self):
        pygame.event.pump()

        steer = int(clamp(round((self.js.get_axis(self.ax_steer) + 1) / 2 * 140 + 25), 25, 165))
        gaz  = 1 if self.js.get_axis(AX_THR) < -DEADZONE else 0
        fren = 1 if self.js.get_axis(AX_RT) > 0.0 else 0

        rb = self.js.get_button(BTN_RB)
        if rb and not self.rb_prev:
            self.reverse = not self.reverse
            self.get_logger().warn(f"VITES -> {'GERI' if self.reverse else 'ILERI'}")
        self.rb_prev = rb

        lb = self.js.get_button(BTN_LB)
        if lb and not self.lb_prev:
            self.otonom = not self.otonom
            self.get_logger().warn(f"MOD -> {'OTONOM' if self.otonom else 'MANUEL'}")
        self.lb_prev = lb

        msg = Int32MultiArray()
        # ACIL DURUM KALDIRILDI (2026-08-24, kullanici karari).
        # Buton artik OKUNMUYOR ve 6. eleman HEP 0 gonderiliyor -> serial_comm
        # bit4'u asla set etmez -> firmware'deki acilDurumModu HIC tetiklenmez.
        # Sebep: yanlislikla basilma / menzil disinda buton titremesi riski.
        # Geri acmak icin: asagidaki 0 yerine self.js.get_button(BTN_B) yaz.
        acil = 0

        msg.data = [steer, gaz, fren, int(self.reverse), int(self.otonom), int(acil)]
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = GamepadNode()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
