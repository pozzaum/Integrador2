#include "gps.h"

// ====== Estado interno (escopo de arquivo) ======
static TinyGPSPlus      s_gps;
static HardwareSerial*  s_port = nullptr;

static double s_target_lat_deg = NAN;
static double s_target_lon_deg = NAN;
static double s_inner_radius_m = 8.0;
static double s_outer_radius_m = 12.0;
static bool   s_inside_radius  = false;  // estado da histerese

static uint32_t s_report_interval_ms = 1000;
static uint32_t s_last_report_ms     = 0;

// ====== Constantes ======
static constexpr double kEarthRadius_m = 6371000.0;

// ====== Helpers internos ======
static bool target_is_valid(void) {
  return isfinite(s_target_lat_deg) && isfinite(s_target_lon_deg);
}

// ====== API ======

void gps_begin(HardwareSerial* port, uint32_t baud, int rxPin, int txPin) {
  s_port = port;
  if (s_port) {
    s_port->begin(baud, SERIAL_8N1, rxPin, txPin);
  }
  // reset de histerese
  s_inside_radius = false;
}

void gps_feed(void) {
  if (!s_port) return;
  while (s_port->available()) {
    s_gps.encode(s_port->read());
  }
}

void gps_set_report_interval_ms(uint32_t ms) { s_report_interval_ms = ms; }
uint32_t gps_get_report_interval_ms(void) { return s_report_interval_ms; }
uint32_t gps_get_last_report_ms(void) { return s_last_report_ms; }

bool gps_has_data(uint32_t minChars) {
  return s_gps.charsProcessed() >= minChars;
}

bool gps_has_fix(void) {
  return s_gps.location.isValid();
}

bool gps_current_location(double* lat_deg, double* lon_deg) {
  if (!lat_deg || !lon_deg) return false;
  if (!s_gps.location.isValid()) return false;
  *lat_deg = s_gps.location.lat();
  *lon_deg = s_gps.location.lng();
  return true;
}

int gps_satellites(void) {
  if (s_gps.satellites.isValid()) {
    return (int)s_gps.satellites.value();
  }
  return -1;
}

double gps_hdop(void) {
  if (s_gps.hdop.isValid()) {
    // TinyGPS++ fornece o HDOP em centésimos (inteiro).
    // Ex.: 95 => 0.95
    return s_gps.hdop.value() / 100.0;
  }
  return -1.0;
}


void gps_set_target(double lat_deg, double lon_deg,
                    double inner_radius_m, double outer_radius_m) {
  s_target_lat_deg = lat_deg;
  s_target_lon_deg = lon_deg;
  s_inner_radius_m = inner_radius_m;
  s_outer_radius_m = outer_radius_m;
  s_inside_radius  = false; // reavalia ao próximo cálculo
}

bool gps_distance_bearing_to_target(double* distance_m, double* bearing_deg) {
  if (!distance_m || !bearing_deg) return false;
  if (!gps_has_fix() || !target_is_valid()) return false;

  double lat, lon;
  if (!gps_current_location(&lat, &lon)) return false;

  *distance_m  = gps_haversine_m(lat, lon, s_target_lat_deg, s_target_lon_deg);
  *bearing_deg = gps_bearing_deg(lat, lon, s_target_lat_deg, s_target_lon_deg);
  return true;
}

bool gps_compute_go_to_heading(double heading_atual_deg, GpsNavCommand* out_cmd) {
  if (!out_cmd) return false;

  double d, brg;
  if (!gps_distance_bearing_to_target(&d, &brg)) {
    return false;
  }

  // Histerese de chegada
  if (!s_inside_radius && d <= s_inner_radius_m) {
    s_inside_radius = true;
  } else if (s_inside_radius && d >= s_outer_radius_m) {
    s_inside_radius = false;
  }

  out_cmd->distance_m          = d;
  out_cmd->bearing_to_goal_deg = gps_wrap360(brg);
  out_cmd->heading_error_deg   = gps_wrap180(brg - gps_wrap360(heading_atual_deg));
  out_cmd->arrived             = s_inside_radius;

  // Atualiza marcador temporal de "relato" (caso use externamente)
  uint32_t now = millis();
  if (now - s_last_report_ms >= s_report_interval_ms) {
    s_last_report_ms = now;
  }
  return true;
}

// ====== Utilidades públicas ======

double gps_deg2rad(double d) { return d * (PI / 180.0); }
double gps_rad2deg(double r) { return r * (180.0 / PI); }

double gps_wrap180(double deg) {
  // Normaliza para [-180, +180]
  double d = fmod(deg + 180.0, 360.0);
  if (d < 0) d += 360.0;
  return d - 180.0;
}

double gps_wrap360(double deg) {
  // Normaliza para [0, 360)
  double d = fmod(deg, 360.0);
  if (d < 0) d += 360.0;
  return d;
}

double gps_haversine_m(double lat1_deg, double lon1_deg,
                       double lat2_deg, double lon2_deg) {
  const double lat1 = gps_deg2rad(lat1_deg);
  const double lon1 = gps_deg2rad(lon1_deg);
  const double lat2 = gps_deg2rad(lat2_deg);
  const double lon2 = gps_deg2rad(lon2_deg);

  const double dlat = lat2 - lat1;
  const double dlon = lon2 - lon1;

  const double a = sin(dlat/2)*sin(dlat/2)
                 + cos(lat1)*cos(lat2)*sin(dlon/2)*sin(dlon/2);
  const double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  return kEarthRadius_m * c;
}

double gps_bearing_deg(double lat1_deg, double lon1_deg,
                       double lat2_deg, double lon2_deg) {
  const double lat1 = gps_deg2rad(lat1_deg);
  const double lat2 = gps_deg2rad(lat2_deg);
  const double dlon = gps_deg2rad(lon2_deg - lon1_deg);

  const double y = sin(dlon) * cos(lat2);
  const double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon);
  const double brg_rad = atan2(y, x);
  return gps_wrap360(gps_rad2deg(brg_rad));
}
