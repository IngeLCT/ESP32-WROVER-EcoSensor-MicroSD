# API y endpoints de EcoSensor WROVER

## Identidad del dispositivo

- `device_id`: `ecosensor01`
- `mdns_hostname`: `ecosensor01`
- acceso local esperado: `http://ecosensor01.local/`

## Regla operativa de tiempo

El equipo **no debe enviar mediciones al servidor** hasta tener una fecha/hora válida.

Flujo esperado:
1. Arranca el ESP32.
2. Si no tiene Wi‑Fi, levanta AP y Web Server de configuración.
3. Cuando ya tiene red, consulta tiempo/configuración al servidor.
4. Si la fecha no es válida, espera sincronización.
5. Con fecha válida, empieza a publicar promedios.

## Endpoints del servidor

### 1) `POST /api/v1/ingest`

Uso: el ESP32 envía lecturas promediadas.

#### Request JSON
```json
{
  "device_id": "ecosensor01",
  "timestamp": "2026-05-07T16:35:00Z",
  "window_s": 60,
  "co2": 523,
  "pm1p0": 1.2,
  "pm2p5": 2.8,
  "pm4p0": 3.4,
  "pm10p0": 4.1,
  "voc": 0.7,
  "nox": 0.2,
  "temp": 24.6,
  "hum": 48.1
}
```

#### Respuesta JSON sugerida
```json
{
  "ok": true,
  "device_id": "ecosensor01",
  "server_time": "2026-05-07T16:35:02Z"
}
```

## 2) `GET /api/v1/device/{device_id}/config`

Uso: el ESP32 consulta configuración operativa publicada por el servidor.

#### Respuesta JSON sugerida
```json
{
  "ok": true,
  "device_id": "ecosensor01",
  "read_interval_s": 5,
  "upload_interval_s": 60,
  "time_required": true
}
```

Notas:
- `upload_interval_s = 60` durante debug.
- en la implementación real se moverá a `300` (5 min).

## 3) `GET /api/v1/device/{device_id}/time`

Uso: el ESP32 obtiene una fecha/hora válida del servidor.

#### Respuesta JSON sugerida
```json
{
  "ok": true,
  "device_id": "ecosensor01",
  "timestamp": "2026-05-07T16:35:00Z",
  "valid": true
}
```

## Endpoints locales del ESP32

## 4) `GET /status`

Uso: diagnóstico local del equipo por navegador o cliente HTTP.

#### Respuesta JSON
```json
{
  "device_id": "ecosensor01",
  "wifi": "connected",
  "ip": "192.168.1.50",
  "mdns": "ecosensor01.local",
  "time_valid": true,
  "sensors": "running",
  "state": "OPERATIONAL",
  "using_saved": true,
  "conn_attempts": 0
}
```

## 5) `GET /lecturas`

Uso: devolver el último promedio disponible en memoria local.

#### Respuesta JSON cuando ya hay lectura
```json
{
  "device_id": "ecosensor01",
  "valid": true,
  "timestamp": "2026-05-07T16:35:00Z",
  "window_s": 60,
  "co2": 523,
  "pm1p0": 1.2,
  "pm2p5": 2.8,
  "pm4p0": 3.4,
  "pm10p0": 4.1,
  "voc": 0.7,
  "nox": 0.2,
  "temp": 24.6,
  "hum": 48.1
}
```

#### Respuesta JSON cuando aún no hay promedio
```json
{
  "device_id": "ecosensor01",
  "valid": false,
  "window_s": 60,
  "time_valid": false,
  "message": "Sin lecturas promediadas disponibles"
}
```

## Convenciones cerradas

- sin autenticación ni token
- nombres simplificados: `temp`, `hum`
- `device_id` fijo igual al mDNS
- promedio actual: 1 minuto
- promedio objetivo de producción: 5 minutos
