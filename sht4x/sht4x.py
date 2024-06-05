import time, sys
import board
import adafruit_sht4x

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity

class SHT4xNode(Node):
    """
    ROS2 publisher node for Adafruit-packaged Sensirion SHT4X-series sensors
    
    The class publishes the temperature and relative humidity read by the
    senso to separate topics.
    NOTE: The class does not populate the variance field, and returns zero,
    the REP-145 convention for unknown variance/covariance.
    """
    def __init__(self) -> None:
        super().__init__("sht45")    # type: ignore
        self.poll_interval = 1.0 # seconds

        i2c_bus = board.I2C()
        # This is exactly the initialization that the sht4x library does
        # Dunno why it needs to be done again outside of the library __init__
        i2c_bus.writeto(address=adafruit_sht4x._SHT4X_DEFAULT_ADDR, 
            buffer=bytearray([adafruit_sht4x._SHT4X_READSERIAL]))
        time.sleep(0.01) # This is necessary, also what the upstream library does
        
        self.sht = adafruit_sht4x.SHT4x(i2c_bus)
        self.get_logger().info(
            f'Found SHT4X with serial number: {hex(self.sht.serial_number)}')

        self.sht.mode = adafruit_sht4x.Mode.NOHEAT_HIGHPRECISION    # type: ignore
        self.get_logger().info(
            f'Current mode is: {adafruit_sht4x.Mode.string[self.sht.mode]}')

        # Initialize temperature messages and create publisher
        self.msg_temp = Temperature()
        self.msg_temp.header.stamp = self.get_clock().now().to_msg()
        self.msg_temp.temperature = float('NaN')
        self.publisher_sht4xtemp = self.create_publisher(msg_type=Temperature, 
            topic='/sht4x/temperature', qos_profile=10)
        
        # Initialize rh messages and create publisher
        self.msg_rh = RelativeHumidity()
        self.msg_rh.header.stamp = self.get_clock().now().to_msg()
        self.msg_rh.relative_humidity = float('NaN')
        self.publisher_sht4xrh = self.create_publisher(msg_type=RelativeHumidity, 
            topic='/sht4x/relative_humidity', qos_profile=10)
        
        self.timer = self.create_timer(self.poll_interval, self.sht4x_callback)

    def sht4x_callback(self) -> None:
        """
        SHT4x timer callback

        Reads the sensor values and publishes the messages at 
        the node polling interval
        """
        self.msg_temp.header.stamp = self.msg_rh.header.stamp = self.get_clock().now().to_msg()
        self.msg_temp.temperature, self.msg_rh.relative_humidity = self.sht.measurements

        self.publisher_sht4xtemp.publish(msg=self.msg_temp)
        self.publisher_sht4xrh.publish(msg=self.msg_rh)

def main(args=None) -> None:
    rclpy.init(args=args)
    sht4x_pub: SHT4xNode = SHT4xNode()

    try:
        rclpy.spin(sht4x_pub) 
    except KeyboardInterrupt:
        print("**** 💀 Ctrl-C detected... killing process ****")
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        print("**** 🪦 process dead ****")
        # Destroy node explicitly, dont wait for GC
        sht4x_pub.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()