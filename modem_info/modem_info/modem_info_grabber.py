# Copyright (C) 2024 InDro Robotics
# Contact: tirth@indrorobotics.com
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#

import rclpy
from rclpy.node import Node
import spur

from router_interfaces.msg import RouterInformation

class ModemInfo(Node):
    def __init__(self):
        super().__init__('modem_info_node')
        self.grab_parameters()
        self.now = self.get_clock().now()
        self.shell = spur.SshShell(
            hostname=self.modem_ip,
            username=self.username,
            password=self.password,
            connect_timeout=5,
            missing_host_key=spur.ssh.MissingHostKey.accept
        )
        self.get_logger().info('Modem Info Grabber Initialized')
        self.get_logger().info('Testing connection to modem...')
        

        try:
            result = self.shell.run(["ls", "-l"])
        except spur.ssh.ConnectionError:
            self.get_logger().error('Connection to modem failed')
            exit()

        self.modem_info_pub = self.create_publisher(RouterInformation, '/router/modem_info', 10)
        self.get_logger().info('Connected...')
        # Publish at 1 Hz
        self.timer = self.create_timer(1, self.timer_callback)
    
    def grab_parameters(self):
        self.declare_parameter('modem_ip', '192.168.42.1')
        self.declare_parameter('modem_user', 'root')
        self.declare_parameter('modem_pass', 'indr0.com')

        self.modem_ip = self.get_parameter('modem_ip').get_parameter_value().string_value
        self.username = self.get_parameter('modem_user').get_parameter_value().string_value
        self.password = self.get_parameter('modem_pass').get_parameter_value().string_value

    def grab_modem_info(self):
        try:
            result = self.shell.run(["cat", "/tmp/status1.file"]) # reading files
            msim = self.shell.run(["cat", "/tmp/msimdata1"]) # reading files
            public_ip = self.shell.run(["cat", "/tmp/ipip"]) # reading files
            wifi = self.shell.run(["cat", "/etc/config/wireless"]) # reading files

        except spur.ssh.ConnectionError:
            self.get_logger().error('Connection to modem failed')
            return
        formatted_output_list = result.output.decode("utf-8").rstrip().split('\n') # decoding, splitting and saving them as arrays
        msim_list = msim.output.decode("utf-8").rstrip().split('\n') # decoding, splitting and saving them as arrays
        public_ip_list = public_ip.output.decode("utf-8").rstrip().split('\n') # decoding, splitting and saving them as arrays
        wifi_list = wifi.output.decode("utf-8").rstrip().split('\n') # decoding, splitting and saving them as arrays


        wifi_connection = wifi_list[25] 
        is_connected = wifi_connection.find("ssid") # checking if the wifi is connected

        get_wifi_ssid = wifi_connection.split('\'')
        

       

    
        modem_info = RouterInformation()
        modem_info.header.stamp = self.now.to_msg()
        for item in formatted_output_list:
            item = formatted_output_list
            modem_info.modem_port=item[0]
            modem_info.device_model=item[4]
            modem_info.csq=item[1]
            modem_info.rssi=item[3]
            modem_info.signal_quality_rsrq=item[17]
            modem_info.signal_power_rsrp=item[18]
            modem_info.sinr=item[32]
            modem_info.cell_info_mcc_mnc=item[11]
            modem_info.cell_id=item[9]
            modem_info.cell_band_info=item[28]
            modem_info.cell_channel=item[25]
            modem_info.cell_pci=item[31]
            modem_info.signal_strength=item[2]
            modem_info.provider=item[5]
            modem_info.temperature=item[29]
            modem_info.protocol=item[30]
            modem_info.network=item[6]

            if is_connected == 8:
                modem_info.is_wifi_connected = "Yes"
                modem_info.wifi_ssid = get_wifi_ssid[1]
            else :
                modem_info.is_wifi_connected = "No"
            
            

        for msim_item in msim_list:
            msim_item = msim_list
            modem_info.sim_imsi=msim_item[2]
            modem_info.imei=msim_item[1]
            modem_info.iccid=msim_item[3]

        for public_ip_info in public_ip_list:
            public_ip_info = public_ip_list
            modem_info.public_ip=public_ip_info[0]
            self.modem_info_pub.publish(modem_info)
        # for item in formatted_output_list:
        #     item = item.split(':')
        #     if item[0].find('device_model') != -1:
        #         modem_info.device_model = item[1].replace('"', '')
        #     elif item[0].find('imei') != -1:
        #         modem_info.imei = int(item[1].replace('"', ''))
        #     elif item[0].find('iccid') != -1:
        #         modem_info.iccid = item[1].replace('"', '')
        #     elif item[0].find('apn') != -1:
        #         modem_info.apn = item[1].replace('"', '')
        #     elif item[0].find('pin') != -1:
        #         modem_info.pin = item[1].replace('"', '')
        #     elif item[0].find('isp') != -1:
        #         modem_info.isp = item[1].replace('"', '')
        #     elif item[0].find('signal') != -1:
        #         modem_info.signal = int(item[1].replace('"', ''))
        #     elif item[0].find('cgreg') != -1:
        #         modem_info.cgreg = int(item[1].replace('"', ''))
        #     elif item[0].find('nwmode') != -1:
        #         modem_info.nwmode = int(item[1].replace('"', ''))
        #     elif item[0].find('scanmode') != -1:
        #         modem_info.scanmode = item[1].replace('"', '')
        #     elif item[0].find('bandval') != -1:
        #         modem_info.bandval = item[1].replace('"', '')
        #     elif item[0].find('ltebandval') != -1:
        #         modem_info.ltebandval = item[1].replace('"', '')

        
        #  self.modem_info_pub.publish(modem_info)


    def timer_callback(self):
        self.now = self.get_clock().now()
        self.grab_modem_info()

def main(args=None):
    rclpy.init(args=args)
    modem_info = ModemInfo()
    rclpy.spin(modem_info)
    modem_info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()