import pyudev
import rclpy
from rclpy.node import Node
import os


class USBMonitor(Node):

    def __init__(self):
        super().__init__('usb_monitor')
        self.contextt = pyudev.Context()
        self.monitor = pyudev.Monitor.from_netlink(self.contextt)
        self.monitor.filter_by(subsystem='usb')
        self.observer = pyudev.MonitorObserver(self.monitor, callback=self.device_event)
        self.observer.start()
        self.get_logger().info("USB Monitor started, waiting for events...")

    def device_event(self, device):
        if (device.action == 'remove') and ('/sys' + device.device_path == os.environ['MAGICIAN_USB_DEVICE_PATH']):
            self.observer.stop()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    usb_monitor = USBMonitor()
    try:
        rclpy.spin(usb_monitor)
    except (KeyboardInterrupt):
        pass
    finally:
        usb_monitor.destroy_node()


if __name__ == '__main__':
    main()
