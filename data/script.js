// EcoSensor - Web server ligero de respaldo
// Lee la última medición desde /api/latest y actualiza la tabla del HTML.
// Sin IndexedDB, sin historial local, sin descarga CSV y sin EventSource/SSE.

const API_LATEST_URL = '/api/latest';
const REFRESH_INTERVAL_MS = 5 * 60 * 1000; // 5 minutos

const SENSOR_FIELDS = [
  { id: 'pm1', keys: ['pm1', 'pm1_0', 'pm1p0', 'pm1p0_ug_m3', 'pm1_0_ug_m3', 'sen55_pm1p0'], decimals: 1 },
  { id: 'pm25', keys: ['pm25', 'pm2_5', 'pm2p5', 'pm25_ug_m3', 'pm2_5_ug_m3', 'pm2p5_ug_m3', 'sen55_pm2p5'], decimals: 1 },
  { id: 'pm4', keys: ['pm4', 'pm4_0', 'pm4p0', 'pm4_ug_m3', 'pm4_0_ug_m3', 'pm4p0_ug_m3', 'sen55_pm4p0'], decimals: 1 },
  { id: 'pm10', keys: ['pm10', 'pm10_0', 'pm10p0', 'pm10_ug_m3', 'pm10_0_ug_m3', 'pm10p0_ug_m3', 'sen55_pm10p0'], decimals: 1 },
  { id: 'voc', keys: ['voc', 'voc_index', 'vocIndex', 'sen55_voc', 'sen55_voc_index'], decimals: 0 },
  { id: 'nox', keys: ['nox', 'nox_index', 'noxIndex', 'sen55_nox', 'sen55_nox_index'], decimals: 0 },
  { id: 'co2', keys: ['co2', 'co2_ppm', 'co2_avg', 'scd41_co2', 'scd41_co2_ppm'], decimals: 0 },
  { id: 'temp', keys: ['temp', 'temperatura', 'temperature', 'temp_c', 'temperature_c', 'sen55_temp', 'scd41_temp'], decimals: 1 },
  { id: 'hume', keys: ['hume', 'hum', 'humedad', 'humedad_relativa', 'humidity', 'humidity_pct', 'relative_humidity', 'rh', 'sen55_hum', 'scd41_hum'], decimals: 1 },
];

function setText(id, value) {
  const element = document.getElementById(id);
  if (!element) return;
  element.textContent = value;
}

function firstDefined(obj, keys) {
  for (const key of keys) {
    if (obj && Object.prototype.hasOwnProperty.call(obj, key)) {
      const value = obj[key];
      if (value !== null && value !== undefined && value !== '') return value;
    }
  }
  return undefined;
}

function normalizePayload(payload) {
  return payload?.latest ?? payload?.data ?? payload?.reading ?? payload?.measurement ?? payload;
}

function formatValue(value, decimals) {
  if (value === null || value === undefined || value === '') return '—';
  const numberValue = Number(value);
  if (Number.isFinite(numberValue)) return numberValue.toFixed(decimals);
  return String(value);
}

function parseTimestamp(value) {
  if (value === null || value === undefined || value === '') return null;

  if (typeof value === 'number') {
    const milliseconds = value < 10000000000 ? value * 1000 : value;
    const date = new Date(milliseconds);
    return Number.isNaN(date.getTime()) ? null : date;
  }

  const date = new Date(String(value));
  return Number.isNaN(date.getTime()) ? null : date;
}

function formatDate(date) {
  return date.toLocaleDateString('es-MX', { day: '2-digit', month: '2-digit', year: 'numeric' });
}

function formatTime(date) {
  return date.toLocaleTimeString('es-MX', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
}

function updateMeasurementTime(data) {
  const timestamp = firstDefined(data, ['ts_utc', 'timestamp', 'datetime', 'date_time', 'fecha_hora', 'fechaHora', 'time_iso', 'iso_time']);
  const parsedTimestamp = parseTimestamp(timestamp);
  if (parsedTimestamp) {
    setText('last-date', formatDate(parsedTimestamp));
    setText('last-time', formatTime(parsedTimestamp));
    return;
  }

  const fecha = firstDefined(data, ['fecha', 'date', 'fecha_medicion']);
  const hora = firstDefined(data, ['hora', 'time', 'hora_medicion', 'hora_local']);

  if (fecha || hora) {
    setText('last-date', fecha ?? '—');
    setText('last-time', hora ?? '—');
    return;
  }

  const now = new Date();
  setText('last-date', formatDate(now));
  setText('last-time', formatTime(now));
}

function updateDeviceTitle(data) {
  const deviceId = firstDefined(data, ['device_id', 'id', 'ID', 'mac', 'MAC']);
  const title = document.querySelector('h2');
  if (title && deviceId) title.textContent = `EcoSensor: ${deviceId}`;
}

function updateTable(data) {
  for (const field of SENSOR_FIELDS) {
    const value = firstDefined(data, field.keys);
    setText(field.id, formatValue(value, field.decimals));
  }
  updateMeasurementTime(data);
  updateDeviceTitle(data);
}

async function getLatestReading() {
  try {
    const response = await fetch(API_LATEST_URL, { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const payload = await response.json();
    const data = normalizePayload(payload);
    updateTable(data);
  } catch (error) {
    console.error('Error al leer /api/latest:', error);
    setText('last-date', 'Sin datos');
    setText('last-time', 'Error al leer /api/latest');
  }
}

function startPolling() {
  getLatestReading();
  window.setInterval(getLatestReading, REFRESH_INTERVAL_MS);
}

window.addEventListener('load', startPolling);
