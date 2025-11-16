#!/usr/bin/env python3
"""
Nodo ROS unificado para procesar datos del UM982 GPS
Suscribe a los tópicos del um982_serial_driver y publica datos procesados

Suscribe a:
  - /gngga (nmea_msgs/Sentence) → Procesa GGA
  - /gnrmc (nmea_msgs/Sentence) → Procesa RMC
  - /uniheading (std_msgs/String) → Procesa UNIHEADING

Publica:
  - /fix (sensor_msgs/NavSatFix) → Posición + altitud desde GGA
  - /odom/rmc (nav_msgs/Odometry) → Velocidad lineal desde RMC
  - /imu/uniheading (sensor_msgs/Imu) → Orientación desde UNIHEADING
  - /uniheading/heading (std_msgs/String) → Heading en grados y radianes

Característica: Si un tópico de entrada no existe, no se publica el correspondiente de salida
"""

import rospy
import math
import csv
import io
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def wrap(a):
    """Ajusta un ángulo a (-π, π]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def heading_to_yaw_enu(heading_deg, offset_deg=-90.0):
    """
    Convierte heading NMEA (0°=Norte, horario) a yaw ENU (0 rad=Este, antihorario),
    aplicando offset de montaje.
    """
    yaw = math.radians(90.0 - heading_deg + offset_deg)
    return wrap(yaw)


def rpy_to_quat(roll, pitch, yaw):
    """RPY (rad) -> quaternion (x,y,z,w)."""
    cr = math.cos(roll / 2.0); sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0); sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0);   sy = math.sin(yaw / 2.0)
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    w = cr*cp*cy + sr*sp*sy
    return (x, y, z, w)


def parse_uniheading_line(s):
    """
    Parse una línea completa del tópico /uniheading (String.data).
    Devuelve (heading, pitch, hdgstddev, ptchstddev) como floats, o None si no se puede parsear.
    """
    if not s:
        return None

    line = s.strip()
    line = line.replace("\\", "").strip()

    if not line.startswith("#UNIHEADINGA"):
        return None

    parts = line.split(";", 1)
    if len(parts) != 2:
        return None
    payload = parts[1].strip()

    try:
        reader = csv.reader(io.StringIO(payload))
        row = next(reader, [])
    except Exception:
        return None

    if len(row) < 8:
        return None

    def to_float_safe(x):
        x = x.strip()
        if len(x) >= 2 and ((x[0] == x[-1] == '"') or (x[0] == x[-1] == "'")):
            x = x[1:-1]
        return float(x)

    try:
        heading = to_float_safe(row[3])
        pitch = to_float_safe(row[4])
        hdgstddev = to_float_safe(row[6])
        ptchstddev = to_float_safe(row[7])
        return (heading, pitch, hdgstddev, ptchstddev)
    except Exception:
        return None


class UM982TopicDriver:
    """
    Driver unificado para procesar todos los datos del UM982
    """
    
    def __init__(self):
        # Parámetros configurables
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.imu_frame_id = rospy.get_param('~imu_frame_id', 'imu_link_2')
        self.offset_deg = float(rospy.get_param('~offset_deg', -90.0))
        self.roll_deg = float(rospy.get_param('~roll_deg', 0.0))
        self.use_only_yaw = bool(rospy.get_param('~use_only_yaw', False))
        
        # Valores de covarianza por tipo de fix (en metros^2)
        self.covariance_type_map = {
            0: 1000000.0,  # No fix
            1: 4.0,         # GPS fix estándar (SPS)
            2: 0.1,         # DGPS
            4: 0.02,        # RTK Fix
            5: 0.5,         # RTK Float
            6: 10.0,        # Estimado
            9: 0.1          # WAAS/SBAS
        }
        
        # Publishers (se crean solo si se necesitan)
        self.fix_pub = None
        self.odom_pub = None
        self.imu_pub = None
        self.hdg_pub = None
        
        # Subscribers
        self.gga_sub = rospy.Subscriber('/gngga', Sentence, 
                                        self.gga_callback, queue_size=50)
        self.rmc_sub = rospy.Subscriber('/gnrmc', Sentence, 
                                        self.rmc_callback, queue_size=50)
        self.uniheading_sub = rospy.Subscriber('/uniheading', String, 
                                               self.uniheading_callback, queue_size=20)
        
        # Estadísticas
        self.stats = {
            'gga_received': 0,
            'gga_published': 0,
            'rmc_received': 0,
            'rmc_published': 0,
            'uniheading_received': 0,
            'uniheading_published': 0,
            'errors': 0
        }
        
        # Timer para mostrar status cada 5 segundos
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self.print_status)
        
        rospy.loginfo("╔════════════════════════════════════════════════════════════════╗")
        rospy.loginfo("║          UM982 TOPIC DRIVER - INICIALIZADO                     ║")
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║ Esperando datos de /gngga, /gnrmc, /uniheading...             ║")
        rospy.loginfo("╚════════════════════════════════════════════════════════════════╝")
    
    def print_status(self, event=None):
        """
        Imprime el estado actual en formato tabla limpio
        """
        rospy.loginfo("╔════════════════════════════════════════════════════════════════╗")
        rospy.loginfo("║                    ESTADO UM982 TOPIC DRIVER                   ║")
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        
        # Estado de publishers
        gga_status = "✓ ACTIVO" if self.fix_pub is not None else "✗ INACTIVO"
        rmc_status = "✓ ACTIVO" if self.odom_pub is not None else "✗ INACTIVO"
        imu_status = "✓ ACTIVO" if self.imu_pub is not None else "✗ INACTIVO"
        
        rospy.loginfo("║ Tópicos de Salida:                                             ║")
        rospy.loginfo("║   /fix            → %-42s ║", gga_status)
        rospy.loginfo("║   /odom/rmc       → %-42s ║", rmc_status)
        rospy.loginfo("║   /imu/uniheading → %-42s ║", imu_status)
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        
        # Estadísticas
        rospy.loginfo("║ Mensajes Procesados:                                           ║")
        rospy.loginfo("║   GGA:        %6d recibidos  →  %6d publicados            ║",
                     self.stats['gga_received'], self.stats['gga_published'])
        rospy.loginfo("║   RMC:        %6d recibidos  →  %6d publicados            ║",
                     self.stats['rmc_received'], self.stats['rmc_published'])
        rospy.loginfo("║   UNIHEADING: %6d recibidos  →  %6d publicados            ║",
                     self.stats['uniheading_received'], self.stats['uniheading_published'])
        
        if self.stats['errors'] > 0:
            rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
            rospy.loginfo("║   ⚠ Errores: %6d                                            ║",
                         self.stats['errors'])
        
        rospy.loginfo("╚════════════════════════════════════════════════════════════════╝")
    
    def gga_callback(self, msg):
        """
        Callback para mensajes GGA
        """
        self.stats['gga_received'] += 1
        
        try:
            sentence = msg.sentence.strip()
            
            if not ('GGA' in sentence and ('GNGGA' in sentence or 'GPGGA' in sentence)):
                return
            
            fix_msg = self.parse_gga(sentence, msg.header.stamp)
            if fix_msg:
                # Crear publisher si no existe
                if self.fix_pub is None:
                    self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
                    rospy.loginfo("✓ Tópico /fix creado y activo")
                
                # Publicar fix
                self.fix_pub.publish(fix_msg)
                self.stats['gga_published'] += 1
        
        except Exception as e:
            self.stats['errors'] += 1
            rospy.logwarn_throttle(10.0, "⚠ Error procesando GGA: %s", e)
    
    def rmc_callback(self, msg):
        """
        Callback para mensajes RMC
        """
        self.stats['rmc_received'] += 1
        
        try:
            sentence = msg.sentence.strip()
            
            if not ('RMC' in sentence and ('GNRMC' in sentence or 'GPRMC' in sentence)):
                return
            
            odom_msg = self.parse_rmc(sentence, msg.header.stamp)
            if odom_msg:
                # Crear publisher si no existe
                if self.odom_pub is None:
                    self.odom_pub = rospy.Publisher('odom/rmc', Odometry, queue_size=10)
                    rospy.loginfo("✓ Tópico /odom/rmc creado y activo")
                
                # Publicar odometría
                self.odom_pub.publish(odom_msg)
                self.stats['rmc_published'] += 1
        
        except Exception as e:
            self.stats['errors'] += 1
            rospy.logwarn_throttle(10.0, "⚠ Error procesando RMC: %s", e)
    
    def uniheading_callback(self, msg):
        """
        Callback para mensajes UNIHEADING
        """
        self.stats['uniheading_received'] += 1
        
        try:
            # Parsear trama UNIHEADINGA
            tup = parse_uniheading_line(msg.data)
            if tup is None:
                return
            
            heading_deg, pitch_deg, hdgstddev_deg, ptchstddev_deg = tup
            
            # Convertir heading a yaw ENU con offset
            yaw_rad = heading_to_yaw_enu(heading_deg, self.offset_deg)
            yaw_deg = math.degrees(yaw_rad)
            
            # Crear publishers si no existen
            if self.hdg_pub is None:
                self.hdg_pub = rospy.Publisher('/uniheading/heading', String, queue_size=10)
                rospy.loginfo("✓ Tópico /uniheading/heading creado y activo")
            
            if self.imu_pub is None:
                self.imu_pub = rospy.Publisher('/imu/uniheading', Imu, queue_size=20)
                rospy.loginfo("✓ Tópico /imu/uniheading creado y activo")
            
            # Publicar heading en grados y radianes
            hdg_msg = String()
            hdg_msg.data = "({:.4f} deg), ({:.6f} rad)".format(yaw_deg, yaw_rad)
            self.hdg_pub.publish(hdg_msg)
            
            # Preparar orientación
            if self.use_only_yaw:
                roll_rad = 0.0
                pitch_rad = 0.0
            else:
                roll_rad = math.radians(self.roll_deg)
                pitch_rad = math.radians(pitch_deg)
            
            # Quaternion de orientación
            qx, qy, qz, qw = rpy_to_quat(roll_rad, pitch_rad, yaw_rad)
            
            # Convertir desviaciones estándar a varianzas (rad²)
            pitch_var = math.radians(ptchstddev_deg)**2
            yaw_var = math.radians(hdgstddev_deg)**2
            roll_var = 9999.0  # grande porque no tenemos stddev de roll
            
            # Crear mensaje IMU
            imu = Imu()
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = self.imu_frame_id
            
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw
            imu.orientation_covariance = [
                roll_var, 0.0,       0.0,
                0.0,      pitch_var, 0.0,
                0.0,      0.0,       yaw_var
            ]
            
            # No tenemos velocidades angulares ni aceleraciones lineales
            imu.angular_velocity.x = 0.0
            imu.angular_velocity.y = 0.0
            imu.angular_velocity.z = 0.0
            imu.angular_velocity_covariance = [-1.0] + [0.0]*8  # -1 => no disponible
            
            imu.linear_acceleration.x = 0.0
            imu.linear_acceleration.y = 0.0
            imu.linear_acceleration.z = 0.0
            imu.linear_acceleration_covariance = [-1.0] + [0.0]*8  # -1 => no disponible
            
            self.imu_pub.publish(imu)
            self.stats['uniheading_published'] += 1
        
        except Exception as e:
            self.stats['errors'] += 1
            rospy.logwarn_throttle(10.0, "⚠ Error procesando UNIHEADING: %s", e)
    
    def parse_gga(self, sentence, timestamp):
        """
        Parsea una sentencia GNGGA y retorna un mensaje NavSatFix
        """
        # Verificar checksum
        if not self.check_nmea_checksum(sentence):
            return None
        
        # Dividir por comas
        parts = sentence.split(',')
        
        if len(parts) < 15:
            return None
        
        # Crear mensaje NavSatFix
        fix = NavSatFix()
        fix.header.stamp = timestamp
        fix.header.frame_id = self.frame_id
        
        try:
            # Calidad del fix
            quality = int(parts[6]) if parts[6] else 0
            
            # Estado del fix
            if quality == 0:
                fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif quality in [1, 6]:
                fix.status.status = NavSatStatus.STATUS_FIX
            elif quality == 2:
                fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif quality in [4, 5, 9]:
                fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                fix.status.status = NavSatStatus.STATUS_FIX
            
            fix.status.service = NavSatStatus.SERVICE_GPS
            
            # Latitud
            if parts[2] and parts[3]:
                lat_str = parts[2]
                lat_deg = float(lat_str[:2])
                lat_min = float(lat_str[2:])
                fix.latitude = lat_deg + lat_min / 60.0
                if parts[3] == 'S':
                    fix.latitude = -fix.latitude
            else:
                fix.latitude = float('nan')
            
            # Longitud
            if parts[4] and parts[5]:
                lon_str = parts[4]
                lon_deg = float(lon_str[:3])
                lon_min = float(lon_str[3:])
                fix.longitude = lon_deg + lon_min / 60.0
                if parts[5] == 'W':
                    fix.longitude = -fix.longitude
            else:
                fix.longitude = float('nan')
            
            # Altitud
            if parts[9]:
                fix.altitude = float(parts[9])
            else:
                fix.altitude = float('nan')
            
            # Covarianza
            hdop = float(parts[8]) if parts[8] else 99.0
            base_covariance = self.covariance_type_map.get(quality, 10.0)
            covariance_value = base_covariance * (hdop ** 2)
            
            fix.position_covariance = [
                covariance_value, 0.0, 0.0,
                0.0, covariance_value, 0.0,
                0.0, 0.0, covariance_value * 2.0
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            return fix
            
        except (ValueError, IndexError) as e:
            rospy.logwarn_throttle(5.0, "Error parseando campos GGA: %s", e)
            return None
    
    def parse_rmc(self, sentence, timestamp):
        """
        Parsea una sentencia GNRMC y retorna Odometry
        """
        # Verificar checksum
        if not self.check_nmea_checksum(sentence):
            return None
        
        # Dividir por comas
        parts = sentence.split(',')
        
        if len(parts) < 10:
            return None
        
        try:
            # Verificar status
            status = parts[2]
            if status != 'A':
                # Datos inválidos
                return None
            
            # Crear mensaje Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp
            odom_msg.header.frame_id = self.frame_id
            odom_msg.child_frame_id = self.child_frame_id
            
            # Velocidad lineal (convertir de nudos a m/s)
            # 1 nudo = 0.514444 m/s
            if parts[7]:
                speed_knots = float(parts[7])
                speed_ms = speed_knots * 0.514444
                odom_msg.twist.twist.linear.x = speed_ms
            else:
                odom_msg.twist.twist.linear.x = 0.0
            
            # vy = 0 (robot diferencial no se mueve lateralmente)
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            
            # Velocidad angular (no disponible en RMC)
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            
            # Covarianza de velocidad
            vel_std = max(0.1, odom_msg.twist.twist.linear.x * 0.05)
            vel_var = vel_std ** 2
            
            odom_msg.twist.covariance = [
                vel_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 9999.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 9999.0
            ]
            
            # Posición (no la usamos del GPS RMC, solo velocidad)
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            
            # Orientación (quaternion identidad)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0
            
            # Covarianza de posición/orientación muy alta (EKF NO las usa)
            odom_msg.pose.covariance = [
                9999.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 9999.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 9999.0
            ]
            
            return odom_msg
            
        except (ValueError, IndexError) as e:
            rospy.logwarn_throttle(5.0, "Error parseando campos RMC: %s", e)
            return None
    
    def check_nmea_checksum(self, sentence):
        """
        Verifica el checksum de una sentencia NMEA
        """
        try:
            parts = sentence.split('*')
            if len(parts) != 2:
                return False
            
            transmitted_checksum = parts[1].strip()[:2]
            data = parts[0][1:]
            calculated_checksum = 0
            for char in data:
                calculated_checksum ^= ord(char)
            
            return f"{calculated_checksum:02X}" == transmitted_checksum.upper()
            
        except Exception:
            return False
    
    def shutdown_hook(self):
        """
        Se ejecuta al cerrar el nodo
        """
        rospy.loginfo("╔════════════════════════════════════════════════════════════════╗")
        rospy.loginfo("║            ESTADÍSTICAS FINALES - UM982 TOPIC DRIVER           ║")
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║   GGA:        %6d recibidos  →  %6d publicados            ║",
                     self.stats['gga_received'], self.stats['gga_published'])
        rospy.loginfo("║   RMC:        %6d recibidos  →  %6d publicados            ║",
                     self.stats['rmc_received'], self.stats['rmc_published'])
        rospy.loginfo("║   UNIHEADING: %6d recibidos  →  %6d publicados            ║",
                     self.stats['uniheading_received'], self.stats['uniheading_published'])
        rospy.loginfo("╠════════════════════════════════════════════════════════════════╣")
        rospy.loginfo("║   Errores totales: %6d                                      ║",
                     self.stats['errors'])
        rospy.loginfo("╚════════════════════════════════════════════════════════════════╝")


def main():
    """
    Función principal
    """
    rospy.init_node('um982_topic_driver', anonymous=False)
    
    try:
        driver = UM982TopicDriver()
        rospy.on_shutdown(driver.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
