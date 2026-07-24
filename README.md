# ESP32-WROVER EcoSensor MicroSD

Firmware ESP-IDF para EcoSensor WROVER con Wi-Fi manager, mediciones ambientales, almacenamiento en MicroSD y OTA local ordenada por el servidor EcoSensor.

## Destino de mediciones push

Al arrancar, el firmware usa `ecosensor.local` y el puerto HTTP `80`. Cuando
`Ecosensor-Servidor-Distribucion` detecta el equipo, envía `push_host` y
`push_port` mediante `POST /time` o `POST /config`. El ESP32 valida el puerto y
usa el destino recibido para `POST /api/measurements/push`.

El host y el puerto permanecen únicamente en RAM porque pueden cambiar cada vez
que inicia el servidor. `GET /status` expone ambos valores para diagnóstico.

## OTA local

Esta versión usa tabla de particiones OTA para flash de 4 MB:

- `otadata` en `0xd000`
- `ota_0` en `0x10000`
- `ota_1` en `0x200000`

El archivo reproducible es `partitions_ota_4mb.csv` y la configuración queda en `sdkconfig.defaults`.

> Importante: los equipos que todavía tengan una imagen antigua con partición Single App Large deben migrarse una vez por USB/cable usando esta tabla OTA. Después de esa primera migración podrán actualizarse por OTA local.

## Versión de firmware

La versión se define en el `CMakeLists.txt` raíz con `set(PROJECT_VER "x.y.z")`, igual que en el proyecto GSM. El firmware la expone como `firmware_version` en:

- `GET /status`
- `GET /diagnostics`
- `GET /debug`

## Endpoints OTA del ESP32

### `POST /ota/update`

Payload esperado:

```json
{
  "device_id": "ecosensor02",
  "version": "1.0.1",
  "firmware_url": "http://192.168.1.97:8765/firmware/ecosensor02/ecosensor02_v1.0.1.bin",
  "sha256": "opcional"
}
```

El endpoint valida:

- que haya red STA con IP;
- que no haya otra OTA en curso;
- que `device_id`, `version` y `firmware_url` existan;
- que `device_id` coincida con el `mdns_hostname` del firmware;
- que la URL sea HTTP local (`http://...`).

Responde de inmediato con `state: queued` y ejecuta la descarga en una tarea independiente.

### `GET /ota/status`

Devuelve:

```json
{
  "ok": true,
  "state": "idle|queued|downloading|writing|success|error|rebooting",
  "current_version": "1.0.1",
  "target_version": "1.0.2",
  "bytes_received": 0,
  "total_bytes": -1,
  "progress_pct": null,
  "last_error": null
}
```

## Rollback

Está habilitado `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`. En el primer arranque de una imagen OTA nueva, `app_main()` marca la app como válida después de un arranque mínimo correcto. La ausencia de SD no provoca rollback: el firmware ya tolera SD no disponible.

## Decisión durante OTA

La OTA corre en una tarea separada y evita múltiples actualizaciones simultáneas. No se detienen sensores ni SD por defecto para no romper la arquitectura actual; la escritura OTA usa particiones flash separadas. Si más adelante se observa contención, se puede agregar una pausa explícita de sensores durante `downloading/writing`.

## Compilar

```bash
source /home/eduardo/tools/esp-idf/export.sh
idf.py build
```

Para la primera migración por USB con OTA:

```bash
idf.py flash
```

O manualmente con los offsets reportados por ESP-IDF:

```bash
python -m esptool --chip esp32 -b 460800 --before default_reset --after hard_reset write_flash \
  --flash_mode dio --flash_size 4MB --flash_freq 40m \
  0x1000 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0xd000 build/ota_data_initial.bin \
  0x10000 build/ESP32-WROVER-EcoSensor-MicroSD.bin
```
