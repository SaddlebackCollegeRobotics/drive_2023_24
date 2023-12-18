import ifcfg
import subprocess

# TODO - IN PROGRESS
def configure_can_bus(self, interface_name: str, bitrate: int):

        interface_dict = ifcfg.interfaces().get(interface_name)
    
        if interface_dict is None:
            self.get_logger().error(f'Interface {interface_name} not found!')
            exit(0)

        if 'UP' in interface_dict.get('flags'):
            self.get_logger().info(f'Interface {interface_name} is already enabled.')
        else:
            subprocess.run(["sudo", "ip", "link", "set",
                            interface_dict, "up", "type", "can",
                            "bitrate", str(bitrate)])
            
            self.get_logger().info(f'Started interface {interface_name}.')

self.configure_can_bus(interface_name='can0', bitrate=1000000)
