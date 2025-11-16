#!/usr/bin/env python3
"""
UM982 Serial Driver
Lee datos NMEA (GGA, RMC y UNIHEADING) desde el puerto serial del UM982
y los publica en tópicos ROS separados.

Publica:
  /gngga          (nmea_msgs/Sentence) → Solo mensajes GNGGA/GPGGA
  /gnrmc          (nmea_msgs/Sentence) → Solo mensajes GNRMC/GPRMC
  /uniheading     (std_msgs/String)    → Tramas crudas #UNIHEADING
"""

import rospy
import serial
from nmea_msgs.msg import Sentence
from std_msgs.msg import String


class UM982SerialDriver:
    """
    Driver para leer el puerto serial del UM982 y publicar mensajes
    """
    
    def __init__(self):
        # Parámetros
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        self.timeout = rospy.get_param('~timeout', 1.0)
        
        # Publishers
        self.gga_pub = rospy.Publisher('/gngga', Sentence, queue_size=50)
        self.rmc_pub = rospy.Publisher('/gnrmc', Sentence, queue_size=50)
        self.uniheading_pub = rospy.Publisher('/uniheading', String, queue_size=20)
        
        # Estadísticas
        self.stats = {
            'gga': 0,
            'rmc': 0,
            'uniheading': 0,
            'total': 0,
            'errors': 0
        }
        
        # Serial port
        self.serial_port = None
        
        # Timer para mostrar status cada 5 segundos
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self.print_status)
        
        rospy.loginfo("╔════════════════════════════════════════════════════════════════╗")
        rospy.loginfo("║         UM982 SERIAL DRIVER - INICIALIZADO                     ║")
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║ Puerto: %-54s ║", self.port)
        rospy.loginfo("║ Baudrate: %-51d ║", self.baud)
        rospy.loginfo("╚════════════════════════════════════════════════════════════════╝")
    
    def connect_serial(self):
        """
        Conecta al puerto serial
        """
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            rospy.loginfo("✓ Puerto serial conectado: %s", self.port)
            return True
        except serial.SerialException as e:
            rospy.logerr("✗ Error conectando puerto serial: %s", e)
            return False
    
    def print_status(self, event=None):
        """
        Imprime el estado actual en formato tabla limpio
        """
        if self.stats['total'] == 0:
            return  # No mostrar si aún no hay datos
        
        rospy.loginfo("╔════════════════════════════════════════════════════════════════╗")
        rospy.loginfo("║               ESTADO UM982 SERIAL DRIVER                       ║")
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║ Mensajes publicados:                                           ║")
        rospy.loginfo("║   /gngga      → %6d mensajes                                ║", 
                     self.stats['gga'])
        rospy.loginfo("║   /gnrmc      → %6d mensajes                                ║",
                     self.stats['rmc'])
        rospy.loginfo("║   /uniheading → %6d mensajes                                ║",
                     self.stats['uniheading'])
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║   Total: %6d  │  Errores: %6d                           ║",
                     self.stats['total'], self.stats['errors'])
        rospy.loginfo("╚════════════════════════════════════════════════════════════════╝")
    
    def process_line(self, line):
        """
        Procesa una línea del puerto serial
        """
        try:
            line = line.strip()
            if not line:
                return
            
            timestamp = rospy.Time.now()
            
            # Mensajes GGA
            if 'GGA' in line and ('GNGGA' in line or 'GPGGA' in line):
                msg = Sentence()
                msg.header.stamp = timestamp
                msg.header.frame_id = 'gps'
                msg.sentence = line
                self.gga_pub.publish(msg)
                self.stats['gga'] += 1
                self.stats['total'] += 1
            
            # Mensajes RMC
            elif 'RMC' in line and ('GNRMC' in line or 'GPRMC' in line):
                msg = Sentence()
                msg.header.stamp = timestamp
                msg.header.frame_id = 'gps'
                msg.sentence = line
                self.rmc_pub.publish(msg)
                self.stats['rmc'] += 1
                self.stats['total'] += 1
            
            # Mensajes UNIHEADING
            elif line.startswith('#UNIHEADING'):
                msg = String()
                msg.data = line
                self.uniheading_pub.publish(msg)
                self.stats['uniheading'] += 1
                self.stats['total'] += 1
        
        except Exception as e:
            self.stats['errors'] += 1
            rospy.logwarn_throttle(10.0, "⚠ Error procesando línea: %s", e)
    
    def run(self):
        """
        Loop principal
        """
        if not self.connect_serial():
            return
        
        rospy.loginfo("✓ Iniciando lectura del puerto serial...")
        
        try:
            while not rospy.is_shutdown():
                if self.serial_port and self.serial_port.in_waiting:
                    try:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore')
                        self.process_line(line)
                    except serial.SerialException as e:
                        rospy.logwarn("⚠ Error leyendo serial: %s", e)
                        rospy.sleep(1.0)
                        if not self.connect_serial():
                            break
                else:
                    rospy.sleep(0.01)
        
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
    
    def shutdown(self):
        """
        Cierra el puerto serial
        """
        rospy.loginfo("╔════════════════════════════════════════════════════════════════╗")
        rospy.loginfo("║        ESTADÍSTICAS FINALES - UM982 SERIAL DRIVER              ║")
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║   /gngga      → %6d mensajes publicados                    ║",
                     self.stats['gga'])
        rospy.loginfo("║   /gnrmc      → %6d mensajes publicados                    ║",
                     self.stats['rmc'])
        rospy.loginfo("║   /uniheading → %6d mensajes publicados                    ║",
                     self.stats['uniheading'])
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║   Total: %6d  │  Errores: %6d                           ║",
                     self.stats['total'], self.stats['errors'])
        rospy.loginfo("╚════════════════════════════════════════════════════════════════╝")
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("✓ Puerto serial cerrado")


def main():
    rospy.init_node('um982_serial_driver', anonymous=False)
    
    try:
        driver = UM982SerialDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
