import subprocess
import ifcfg

def configure_test(interface_name: str, bitrate: int):
    
    interface = ifcfg.interfaces().get(interface_name)
    
    if interface is None:
        print(f'Interface {interface_name} not found')
        exit(0)

    if 'UP' in interface.get('flags'):
        print(f'Interface {interface_name} is already up')
    else:
        subprocess.run(["sudo", "ip", "link", "set",
                         interface, "up", "type", "can",
                           "bitrate", str(bitrate)])
        
        print(f'Started interface {interface_name}')


def print_flags():
     
    for name, interface in ifcfg.interfaces().items():
        print(name, interface.get('flags'))


def configure_can_bus(interface: str, bitrate: int):

        found = False
        for name, _ in ifcfg.interfaces().items():
            if name == interface:
                found = True
                print("CAN bus found")
                break  

        if not found:  
            print("CAN bus not found")
            exit(0)

        subprocess.run(["sudo", "ip", "link", "set", interface, "up", "type", "can", "bitrate", str(bitrate)])



def main(args=None):
    
    # print_flags()
    configure_test(interface_name='can0', bitrate=1000000)
    # configure_can_bus(interface='can0', bitrate=1000000)



if __name__ == '__main__':
    main()

