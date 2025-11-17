# UM982 Driver ROS 1 Noetic

Driver ROS completo para el receptor GNSS **Unicore UM982** con soporte para RTK, NTRIP y dual-antenna heading.

![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Python](https://img.shields.io/badge/Python-3.x-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 📋 Descripción

Este paquete proporciona una solución completa para integrar el UM982 en sistemas ROS, incluyendo:

-  Lectura de puerto serial del UM982
-  Procesamiento de mensajes NMEA (GGA, RMC)
-  Soporte para dual-antenna heading (UNIHEADING)
-  Inyección de correcciones RTK vía NTRIP
-  Publicación de tópicos estándar de ROS

## ✨ Características

-  **Modular**: Arquitectura de 2 nodos separados (serial + procesamiento)
-  **RTK Ready**: Soporte nativo para correcciones NTRIP
-  **Alta frecuencia**: 20 Hz para GGA/RMC, 1 Hz para UNIHEADING
-  **Salida limpia**: Dashboard tipo tabla para monitoreo fácil
-  **Robusto**: Reconexión automática y manejo de errores
-  **Flexible**: Los tópicos se crean solo si hay datos disponibles

## 📦 Estructura del Paquete

```
um982_driver/
├── scripts/
│   ├── um982_serial_driver.py    # Lee puerto serial y publica datos crudos
│   └── um982_topic_driver.py     # Procesa y convierte a mensajes ROS estándar
├── sh/
│   └── str2str_ntrip.sh          # Inyecta correcciones RTCM vía NTRIP
├── launch/
│   └── launch_um982_driver.launch # Launch file principal
├── README.md
├── Usage.md
└── package.xml
```

## 🔧 Dependencias

### Sistema
```bash
sudo apt-get install rtklib
```

### ROS
```bash
sudo apt-get install ros-noetic-nmea-msgs
```

### Paquete ROS
- `rospy`
- `std_msgs`
- `sensor_msgs`
- `nav_msgs`
- `nmea_msgs`

## 🚀 Instalación

1. **Clonar en tu workspace:**
```bash
cd ~/catkin_ws/src
git clone https://github.com/tu-usuario/um982_driver.git
```

2. **Instalar dependencias:**
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Compilar:**
```bash
catkin_make
source devel/setup.bash
```

4. **Dar permisos de ejecución:**
```bash
cd ~/catkin_ws/src/um982_driver/scripts
chmod +x *.py *.sh
```

## 📡 Tópicos

### Tópicos de Entrada (internos)
Publicados por `um982_serial_driver`:

| Tópico | Tipo | Frecuencia | Descripción |
|--------|------|------------|-------------|
| `/gngga` | `nmea_msgs/Sentence` | 20 Hz | Mensajes GNGGA/GPGGA |
| `/gnrmc` | `nmea_msgs/Sentence` | 20 Hz | Mensajes GNRMC/GPRMC |
| `/uniheading` | `std_msgs/String` | 1 Hz | Tramas #UNIHEADING crudas |

### Tópicos de Salida (estándar ROS)
Publicados por `um982_topic_driver`:

| Tópico | Tipo | Frecuencia | Descripción |
|--------|------|------------|-------------|
| `/fix` | `sensor_msgs/NavSatFix` | 20 Hz | Posición GNSS (lat, lon, alt) |
| `/odom/rmc` | `nav_msgs/Odometry` | 20 Hz | Velocidad lineal desde RMC |
| `/imu/uniheading` | `sensor_msgs/Imu` | 1 Hz | Orientación desde dual-antenna |
| `/uniheading/heading` | `std_msgs/String` | 1 Hz | Heading en grados y radianes |

## 🎮 Uso

### Lanzamiento básico

```bash
roslaunch um982_driver launch_um982_driver.launch
```

### Con parámetros personalizados

```bash
roslaunch um982_driver launch_um982_driver.launch \
  serial_port:=/dev/ttyUSB0 \
  baud:=115200 \
  enable_ntrip:=true
```

### Sin NTRIP (solo GPS estándar)

```bash
roslaunch um982_driver launch_um982_driver.launch enable_ntrip:=false
```

## ⚙️ Configuración

### Parámetros del Launch File

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyUSB0` | Puerto serial del UM982 |
| `serial_port_ntrip` | string | `ttyUSB0` | Puerto para NTRIP (sin /dev/) |
| `baud` | int | `115200` | Velocidad del puerto serial |
| `ntrip_user` | string | - | Usuario NTRIP |
| `ntrip_host` | string | - | Host del caster NTRIP |
| `ntrip_port` | int | `2101` | Puerto NTRIP |
| `ntrip_mount` | string | - | Mountpoint NTRIP |
| `enable_ntrip` | bool | `true` | Habilitar inyección NTRIP |
| `enable_serial_driver` | bool | `true` | Habilitar driver serial |
| `enable_topic_driver` | bool | `true` | Habilitar procesador de tópicos |
| `frame_id` | string | `odom` | Frame ID para odometría |
| `child_frame_id` | string | `base_link` | Child frame ID |
| `imu_frame_id` | string | `imu_link_2` | Frame ID para IMU |
| `offset_deg` | float | `-90.0` | Offset de montaje del heading |
| `roll_deg` | float | `0.0` | Roll fijo (si se usa) |
| `use_only_yaw` | bool | `false` | Usar solo yaw (ignorar pitch) |

### Configuración de NTRIP

Edita el archivo `launch/launch_um982_driver.launch`:

```xml
<arg name="ntrip_user" default="tu_email@example.com" />
<arg name="ntrip_host" default="tu.caster.ntrip.com" />
<arg name="ntrip_port" default="2101" />
<arg name="ntrip_mount" default="TU_MOUNTPOINT" />
```

### Ajuste de Heading Offset

Si tu antena dual está montada en una orientación diferente:

```bash
roslaunch um982_driver launch_um982_driver.launch offset_deg:=-90.0
```

- `0°`: Antenas apuntan al Este
- `-90°`: Antenas apuntan al Norte (default)
- `90°`: Antenas apuntan al Sur
- `180°`: Antenas apuntan al Oeste

## 📊 Monitoreo

### Visualizar estado en terminal

Los nodos muestran automáticamente tablas de estado cada 5 segundos:

```
╔════════════════════════════════════════════════════════════════╗
║                    ESTADO UM982 TOPIC DRIVER                   ║
╠════════════════════════════════════════════════════════════════╣
║ Tópicos de Salida:                                             ║
║   /fix            → ✓ ACTIVO                                   ║
║   /odom/rmc       → ✓ ACTIVO                                   ║
║   /imu/uniheading → ✓ ACTIVO                                   ║
╠════════════════════════════════════════════════════════════════╣
║ Mensajes Procesados:                                           ║
║   GGA:           500 recibidos  →     500 publicados           ║
║   RMC:           500 recibidos  →     500 publicados           ║
║   UNIHEADING:     25 recibidos  →      25 publicados           ║
╚════════════════════════════════════════════════════════════════╝
```

### Verificar tópicos

```bash
rostopic list
rostopic hz /fix
rostopic echo /fix
```

### Visualizar en RViz

```bash
rosrun rviz rviz
```

Agregar displays:
- `NavSatFix` → `/fix`
- `Odometry` → `/odom/rmc`
- `Imu` → `/imu/uniheading`

## 🔍 Troubleshooting

### El puerto serial no se abre

```bash
# Verificar permisos
sudo usermod -a -G dialout $USER
# Cerrar sesión y volver a entrar

# Verificar puerto
ls -l /dev/ttyUSB*
```

### NTRIP no conecta

```bash
# Verificar que str2str funciona manualmente
str2str -in ntrip://user@host:port/mount -out serial://ttyUSB0:115200:8:n:1

# Verificar conectividad
ping tu.caster.ntrip.com
```

### No aparecen tópicos de salida

- Verifica que el serial driver esté recibiendo datos:
```bash
rostopic echo /gngga
```

- Si no hay datos, verifica la conexión del UM982 y su configuración

### Heading incorrecto

Ajusta el `offset_deg` según la orientación de tus antenas:
```bash
roslaunch um982_driver launch_um982_driver.launch offset_deg:=0.0
```

## 📚 Documentación Adicional

- [Manual del UM982](https://en.unicorecomm.com/products/detail/24)
- [Protocolo NMEA](http://www.gpsinformation.org/dale/nmea.htm)
- [NTRIP Protocol](https://www.use-snip.com/kb/knowledge-base/ntrip-rev1-versus-rev2/)

## 🤝 Contribución

Las contribuciones son bienvenidas! Por favor:

1. Fork el proyecto
2. Crea tu feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push al branch (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📝 Changelog

### v1.0.0 (2025-11-14)
- ✨ Release inicial
- ✅ Soporte completo para GGA, RMC y UNIHEADING
- ✅ Integración NTRIP
- ✅ Dashboard de estado limpio
- ✅ Arquitectura modular de 2 nodos

## 📄 Licencia

MIT License - Ver [LICENSE](LICENSE) para más detalles.

---

## 👥 Autores

**Pablo Vallejos**
- 📧 contacto.pablovallejos@gmail.com

**Manuel Molina**
- 📧 contacto.manuelmolina@gmail.com

---

<div align="center">

**⭐ Si este proyecto te resultó útil, considera darle una estrella en GitHub ⭐**

Hecho con ❤️ por Pablo Vallejos y Manuel Molina

</div>
