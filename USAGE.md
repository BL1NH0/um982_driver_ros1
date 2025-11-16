# UM982 Driver - Guía de Uso Detallada

Esta guía proporciona instrucciones detalladas sobre cómo usar el driver UM982 en diferentes escenarios.

## 📑 Tabla de Contenidos

1. [Inicio Rápido](#inicio-rápido)
2. [Configuración del Hardware](#configuración-del-hardware)
3. [Escenarios de Uso](#escenarios-de-uso)
4. [Integración con Robot Localization](#integración-con-robot-localization)
5. [Calibración y Ajustes](#calibración-y-ajustes)
6. [Visualización](#visualización)
7. [Debugging](#debugging)

---

## 🚀 Inicio Rápido

### 1. Primera Conexión

```bash
# Verificar que el UM982 está conectado
ls /dev/ttyUSB*

# Dar permisos al usuario
sudo usermod -a -G dialout $USER
# Reiniciar sesión

# Lanzar driver básico (sin NTRIP)
roslaunch um982_driver launch_um982_driver.launch enable_ntrip:=false
```

### 2. Con Correcciones RTK (NTRIP)

```bash
# Editar credenciales NTRIP en el launch file
nano ~/catkin_ws/src/um982_driver/launch/launch_um982_driver.launch

# Lanzar con NTRIP habilitado
roslaunch um982_driver launch_um982_driver.launch
```

### 3. Verificar Funcionamiento

```bash
# Terminal 1: Lanzar driver
roslaunch um982_driver launch_um982_driver.launch

# Terminal 2: Ver tópicos disponibles
rostopic list

# Terminal 3: Monitorear posición
rostopic echo /fix

# Terminal 4: Monitorear velocidad
rostopic echo /odom/rmc

# Terminal 5: Monitorear heading
rostopic echo /imu/uniheading
```

---

## 🔌 Configuración del Hardware

### Conexión Física

#### UM982 Single Antenna (solo GPS)
```
UM982          PC
======         ====
TX    -------> RX (USB-Serial)
RX    <------- TX (USB-Serial)
GND   -------> GND
```

#### UM982 Dual Antenna (GPS + Heading)
```
Antena 1 (Main) --> Conector GNSS1
Antena 2 (Aux)  --> Conector GNSS2

Distancia recomendada entre antenas: > 30 cm
Alineación: En línea con el eje longitudinal del robot
```

### Configuración del UM982

Conectar via serial y configurar:

```bash
# Conectar con minicom o screen
screen /dev/ttyUSB0 115200

# Comandos de configuración (copiar y pegar)
CONFIG SIGNALGROUP 2
MODE ROVER SURVEY 30 2.000
GNGGA 0.05
GNRMC 0.05
UNIHEADINGA 1

# Para dual-antenna
CONFIG HEADING AUTO 0.3 0.3

# Guardar configuración
SAVECONFIG
```

---

## 🎯 Escenarios de Uso

### Escenario 1: GPS Básico (Sin RTK)

**Caso de uso:** Navegación outdoor de baja precisión

```bash
roslaunch um982_driver launch_um982_driver.launch \
  enable_ntrip:=false
```

**Precisión esperada:** 2-5 metros

**Tópicos disponibles:**
- `/fix` - Posición GPS
- `/odom/rmc` - Velocidad

---

### Escenario 2: GPS con RTK (Alta Precisión)

**Caso de uso:** Agricultura de precisión, topografía, mapping

```bash
roslaunch um982_driver launch_um982_driver.launch \
  ntrip_user:="tu_email@example.com" \
  ntrip_host:="tu.caster.com" \
  ntrip_port:=2101 \
  ntrip_mount:="TU_MOUNTPOINT"
```

**Precisión esperada:** 
- RTK Fixed: 1-2 cm
- RTK Float: 10-50 cm

**Tópicos disponibles:**
- `/fix` - Posición RTK
- `/odom/rmc` - Velocidad

**Verificar calidad RTK:**
```bash
rostopic echo /fix | grep status
# 0 = No Fix
# 1 = GPS Standard
# 2 = DGPS/RTK Float
# Valores altos = RTK Fixed
```

---

### Escenario 3: GPS + Dual Antenna Heading

**Caso de uso:** Robots autónomos que necesitan orientación precisa

```bash
roslaunch um982_driver launch_um982_driver.launch \
  enable_ntrip:=true \
  offset_deg:=-90.0
```

**Precisión esperada:**
- Posición: 1-2 cm (RTK)
- Heading: 0.2° (con antenas a 1m de distancia)

**Tópicos disponibles:**
- `/fix` - Posición RTK
- `/odom/rmc` - Velocidad
- `/imu/uniheading` - Orientación
- `/uniheading/heading` - Heading en grados

**Ajustar offset según montaje:**
```bash
# Si las antenas apuntan al Norte del robot
roslaunch um982_driver launch_um982_driver.launch offset_deg:=-90.0

# Si las antenas apuntan al Este del robot
roslaunch um982_driver launch_um982_driver.launch offset_deg:=0.0

# Si las antenas apuntan al Sur del robot
roslaunch um982_driver launch_um982_driver.launch offset_deg:=90.0
```

---

### Escenario 4: Solo Velocidad (Sin Posición)

**Caso de uso:** Fusion con otros sensores, solo necesitas velocidad

```bash
roslaunch um982_driver launch_um982_driver.launch \
  enable_serial_driver:=true \
  enable_topic_driver:=true
```

Luego en tu código:
```python
import rospy
from nav_msgs.msg import Odometry

def velocity_callback(msg):
    vx = msg.twist.twist.linear.x  # Velocidad en m/s
    print(f"Velocidad: {vx:.2f} m/s")

rospy.Subscriber('/odom/rmc', Odometry, velocity_callback)
```

---

## 🤖 Integración con Robot Localization

### Configuración EKF con UM982

Crear `robot_localization.yaml`:

```yaml
frequency: 30

two_d_mode: true

publish_tf: true
publish_acceleration: false

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

# Fusión de sensores
odom0: /odom/wheels  # Odometría de ruedas
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,  # vx, vy
               false, false, true,   # vyaw
               false, false, false]

odom1: /odom/rmc  # GPS Velocity
odom1_config: [false, false, false,
               false, false, false,
               true,  false, false,  # solo vx
               false, false, false,
               false, false, false]

imu0: /imu/uniheading  # Heading del UM982
imu0_config: [false, false, false,
              false, false, true,   # solo yaw
              false, false, false,
              false, false, false,
              false, false, false]

# Para GPS absoluto (navsat_transform)
navsat_transform:
  frequency: 10
  magnetic_declination_radians: 0.0
  yaw_offset: 0.0
  broadcast_utm_transform: true
```

### Launch con robot_localization

```xml
<launch>
  <!-- UM982 Driver -->
  <include file="$(find um982_driver)/launch/launch_um982_driver.launch" />
  
  <!-- Robot Localization EKF -->
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_local">
    <rosparam command="load" file="$(find tu_paquete)/config/robot_localization.yaml" />
  </node>
  
  <!-- Navsat Transform (GPS a odometría local) -->
  <node pkg="robot_localization" type="navsat_transform_node" name="navsat_transform">
    <remap from="/gps/fix" to="/fix" />
    <remap from="/imu/data" to="/imu/uniheading" />
    <remap from="/odometry/filtered" to="/odometry/local" />
  </node>
</launch>
```

---

## 🎛️ Calibración y Ajustes

### Calibrar Offset de Heading

1. **Colocar robot apuntando al Norte magnético**
2. **Leer heading actual:**
```bash
rostopic echo /uniheading/heading
```

3. **Calcular offset:**
```
offset_necesario = heading_actual - 90.0
```

4. **Aplicar offset:**
```bash
roslaunch um982_driver launch_um982_driver.launch offset_deg:=<offset_necesario>
```

### Verificar Calidad de Señal

```bash
# Script para monitorear calidad
rostopic echo /fix | grep -E "status|covariance"
```

Interpretación:
- `covariance < 0.1`: Excelente (RTK Fixed)
- `covariance < 1.0`: Bueno (RTK Float)
- `covariance < 10.0`: Regular (DGPS)
- `covariance > 10.0`: Malo (GPS estándar)

---

## 📺 Visualización

### RViz Setup

```bash
# Terminal 1: Lanzar driver
roslaunch um982_driver launch_um982_driver.launch

# Terminal 2: Lanzar RViz
rosrun rviz rviz
```

**Configuración RViz:**

1. **Fixed Frame:** `odom` o `map`

2. **Agregar displays:**
   - **NavSatFix** (topic: `/fix`)
     - Color: Verde
     - Alpha: 0.8
   
   - **Odometry** (topic: `/odom/rmc`)
     - Keep: 100
     - Shape: Arrow
     - Color: Azul
   
   - **Imu** (topic: `/imu/uniheading`)
     - Show Axes: true
     - Axes Length: 2.0

3. **Guardar configuración:** `File → Save Config As...`

### Plotjuggler

Para graficar datos en tiempo real:

```bash
sudo apt install ros-noetic-plotjuggler-ros
rosrun plotjuggler plotjuggler
```

Streams a graficar:
- `/fix/latitude`
- `/fix/longitude`
- `/odom/rmc/twist/twist/linear/x`
- `/imu/uniheading/orientation`

---

## 🐛 Debugging

### Verificar Pipeline Completo

```bash
# Script de verificación
#!/bin/bash

echo "=== Verificando UM982 Driver ==="

echo -n "Serial driver: "
rostopic hz /gngga 2>&1 | grep -q "average rate" && echo "✓ OK" || echo "✗ FAIL"

echo -n "Topic driver: "
rostopic hz /fix 2>&1 | grep -q "average rate" && echo "✓ OK" || echo "✗ FAIL"

echo -n "NTRIP: "
rostopic echo /fix -n 1 | grep -q "status: 2" && echo "✓ RTK" || echo "⚠ No RTK"

echo -n "Heading: "
rostopic hz /imu/uniheading 2>&1 | grep -q "average rate" && echo "✓ OK" || echo "✗ FAIL"
```

### Logs Detallados

```bash
# Habilitar logs de debug
export ROSCONSOLE_CONFIG_FILE=/path/to/debug.conf

# debug.conf:
log4j.logger.ros=DEBUG
log4j.logger.ros.um982_driver=DEBUG
```

### Captura de Datos

Para reportar problemas:

```bash
# Grabar todos los tópicos
rosbag record -a -O um982_debug.bag

# Grabar solo tópicos relevantes
rosbag record /gngga /gnrmc /uniheading /fix /odom/rmc /imu/uniheading
```

---

## 📞 Soporte

- **Issues:** [GitHub Issues](https://github.com/tu-usuario/um982_driver/issues)
- **Documentación:** [README.md](README.md)
- **Wiki:** [GitHub Wiki](https://github.com/tu-usuario/um982_driver/wiki)

---

## 📚 Referencias

- [UM982 User Manual](https://en.unicorecomm.com/products/detail/24)
- [ROS Navigation Stack](http://wiki.ros.org/navigation)
- [Robot Localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [NMEA Protocol](http://www.gpsinformation.org/dale/nmea.htm)
