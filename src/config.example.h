// Connectivity.
#define HOSTNAME "TigoTell"

// Hardware pins.
#define TX_PIN 17
#define RX_PIN 18
#define EN_PIN 21

// InfluxDB stats reporting.
const IPAddress kInfluxHost(192, 168, 8, 1); // Influx DB Host
constexpr uint16_t kInfluxPort = 8096;       // Influx DB UDP Port
// InfluxDB stats push interval.
#define PUSH_STATS_INTERVAL_IN_S 10
const char *MEASUREMENT_NAME = "tigotell_v0";

// Minimum time between outputs for a given node.
#define MIN_SECONDS_BETWEEN_NODE_UPDATES 10
#define MIN_SECONDS_BETWEEN_NODETABLE_UPDATES 60
