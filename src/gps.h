#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#ifdef __cplusplus
extern "C" {
#endif

// Saída do modo "vá ao ponto por rumo"
typedef struct {
  double heading_error_deg;     // erro de rumo (alvo - heading_atual) em [-180, +180]
  double distance_m;            // distância até o alvo (m)
  double bearing_to_goal_deg;   // rumo absoluto até o alvo [0, 360)
  bool   arrived;               // chegou (considera histerese)
} GpsNavCommand;

// ============ Inicialização / ciclo ============

// Inicializa o módulo GPS em 'port' (e.g., Serial1), com 'baud' e pinos RX/TX.
// Ex.: gps_begin(Serial1, 9600, 16, 17);
void gps_begin(HardwareSerial* port, uint32_t baud, int rxPin, int txPin);

// Alimente o parser com os bytes disponíveis (chame a cada loop).
void gps_feed(void);

// Opcional: para diagnósticos simples (sem prints).
// Pode ser usado para rate limit externo de logs, se quiser.
void gps_set_report_interval_ms(uint32_t ms);
uint32_t gps_get_report_interval_ms(void);
uint32_t gps_get_last_report_ms(void);

// ============ Estado / leituras de alto nível ============

// Verdadeiro se há fluxo de caracteres (diagnóstico de ligação RX/TX/GND).
bool gps_has_data(uint32_t minChars);

// Verdadeiro se há fix válido (posição atual confiável).
bool gps_has_fix(void);

// Obtém posição atual (graus). Retorna true se válida.
bool gps_current_location(double* lat_deg, double* lon_deg);

// Acesso a métricas simples (retorna -1 se inválido)
int   gps_satellites(void);   // número de satélites
double gps_hdop(void);        // HDOP (quanto menor, melhor)

// ============ Alvo e navegação ============

// Define alvo (lat/lon em graus) e os raios de chegada com histerese (m).
// inner_radius_m: limiar para marcar "chegou"
// outer_radius_m: limiar para "sair" do estado de chegada (evita oscilações)
void gps_set_target(double lat_deg, double lon_deg,
                    double inner_radius_m, double outer_radius_m);

// Obtém distância (m) e bearing (graus [0,360)) até o alvo a partir da posição atual.
// Retorna false se não houver fix ou alvo não definido.
bool gps_distance_bearing_to_target(double* distance_m, double* bearing_deg);

// Computa comando "vá ao ponto por rumo" dado um heading atual (graus).
// 'heading_atual_deg' tipicamente vem de IMU/magnetômetro (ou COG do GPS se em movimento).
// Retorna false se não houver fix ou alvo inválido.
bool gps_compute_go_to_heading(double heading_atual_deg, GpsNavCommand* out_cmd);

// ============ Utilidades (se precisar externamente) ============

// Converte graus->radianos e vice-versa
double gps_deg2rad(double d);
double gps_rad2deg(double r);

// Normalizações de ângulo
double gps_wrap180(double deg);   // [-180, +180]
double gps_wrap360(double deg);   // [0, 360)

// Haversine (m) e bearing (deg) entre dois pontos (graus)
double gps_haversine_m(double lat1_deg, double lon1_deg,
                       double lat2_deg, double lon2_deg);
double gps_bearing_deg(double lat1_deg, double lon1_deg,
                       double lat2_deg, double lon2_deg);

#ifdef __cplusplus
}
#endif


#endif // GPS_H