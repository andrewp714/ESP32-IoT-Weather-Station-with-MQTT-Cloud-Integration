// Include MQTT client library for ESP-IDF MQTT functionality
#include <mqtt_client.h>
// Include custom MQTT header for function declarations
#include "mqtt.h"

// Declare a static MQTT client handle to manage the connection
static esp_mqtt_client_handle_t client;

// Event handler for MQTT events (e.g., connection, disconnection)
// Parameters: handler_args (user data), base (event base), event_id (specific event), event_data (event details)
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    // Cast event_data to MQTT event handle
    esp_mqtt_event_handle_t event = event_data;
    
    // Handle different MQTT events based on event_id
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            // Log successful connection to MQTT broker
            printf("MQTT connected\n");
            break;
        case MQTT_EVENT_DISCONNECTED:
            // Log disconnection from MQTT broker
            printf("MQTT disconnected\n");
            break;
        default:
            // Ignore other MQTT events (e.g., published, subscribed)
            break;
    }
}

// Initialize MQTT client with configuration and start connection
void mqtt_init(void) {
    // Configure MQTT client with broker details
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.hivemq.com", // Public MQTT broker URI
        .broker.address.port = 1883,                      // Standard MQTT port
    };
    
    // Initialize MQTT client with the configuration
    client = esp_mqtt_client_init(&mqtt_cfg);
    
    // Register event handler for all MQTT events
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // Start the MQTT client to connect to the broker
    esp_mqtt_client_start(client);
}

// Publish data to an MQTT topic
// Parameters: topic (MQTT topic string), data (message payload)
void mqtt_publish(const char *topic, const char *data) {
    // Publish message to specified topic with QoS 1 (at least once delivery)
    // Parameters: client, topic, data, message length (0 for null-terminated), QoS (1), retain (0)
    esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
}
