#!/bin/bash
# -----------------------------------------------------------------------------
# str2str_ntrip.sh
# Script que inyecta correcciones RTCM desde NTRIP al puerto serial del UM982
# Diseñado para trabajar con roslaunch y parámetros ROS
# -----------------------------------------------------------------------------

set -e

# Función para manejar señales de terminación
function cleanup {
    if [ ! -z "$STR2STR_PID" ] && kill -0 "$STR2STR_PID" 2>/dev/null; then
        kill "$STR2STR_PID" || true
        wait "$STR2STR_PID" 2>/dev/null || true
    fi
    exit 0
}

# Capturar señales
trap cleanup SIGINT SIGTERM EXIT

# Leer parámetros desde ROS
NTRIP_URL=$(rosparam get /ntrip_rtcm_injector/ntrip_url 2>/dev/null || echo "")
SERIAL_OUT=$(rosparam get /ntrip_rtcm_injector/serial_out 2>/dev/null || echo "")

# Validar parámetros
if [ -z "$NTRIP_URL" ] || [ -z "$SERIAL_OUT" ]; then
    echo "[NTRIP] ✗ ERROR: Parámetros ROS no encontrados"
    exit 1
fi

# Verificar que str2str está instalado
if ! command -v str2str &> /dev/null; then
    echo "[NTRIP] ✗ ERROR: str2str no instalado (apt install rtklib)"
    exit 1
fi

# Extraer info de la URL para mostrar
NTRIP_HOST=$(echo "$NTRIP_URL" | sed -n 's|.*@\([^:]*\):.*|\1|p')
NTRIP_MOUNT=$(echo "$NTRIP_URL" | sed -n 's|.*/\([^/]*\)$|\1|p')

echo "[NTRIP] ✓ Conectando a $NTRIP_HOST/$NTRIP_MOUNT..."

# Ejecutar str2str en foreground, redirigir su salida
str2str -in "$NTRIP_URL" -out "$SERIAL_OUT" 2>&1 | while IFS= read -r line; do
    # Filtrar mensajes de str2str para hacerlos más limpios
    if [[ "$line" == *"[CC---]"* ]]; then
        echo "[NTRIP] ✓ Conectado y recibiendo datos"
    elif [[ "$line" == *"timeout"* ]] || [[ "$line" == *"[WC---]"* ]]; then
        echo "[NTRIP] ⚠ Timeout - reconectando..."
    elif [[ "$line" == *"stream server stop"* ]]; then
        echo "[NTRIP] ✗ Desconectado"
    fi
done &

STR2STR_PID=$!

# Esperar a que el proceso termine
wait "$STR2STR_PID"
