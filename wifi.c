// Include necessary ESP-IDF libraries for Wi-Fi and event handling
#include "esp_wifi.h"        // Provides Wi-Fi-related functions and configurations
#include "esp_event.h"       // Handles system events like Wi-Fi connection and IP assignment
#include "nvs_flash.h"       // Non-volatile storage for saving Wi-Fi credentials and other data
#include "esp_log.h"         // Logging functions for debugging and status output

// Define Wi-Fi credentials as macros for easy configuration
#define WIFI_SSID "YOUR_SSID"    // Wi-Fi network name (SSID)
#define WIFI_PASS "YOUR_PASSWORD"      // Wi-Fi password

// Event handler function to manage Wi-Fi and IP-related events
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    // Check if the event is a Wi-Fi event and the station (STA) has started
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Initiate connection to the configured Wi-Fi network
    }
    // Check if the event is an IP event and the station has received an IP address
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        printf("Connected to Wi-Fi\n"); // Print success message
    }
    // Check if the event is a Wi-Fi event and the station has disconnected
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("Disconnected. Reconnecting...\n"); // Notify disconnection
        esp_wifi_connect(); // Attempt to reconnect to the Wi-Fi network
    }
}

// Function to initialize and configure Wi-Fi in station mode
void wifi_init(void) {
    // Initialize non-volatile storage (NVS) to store Wi-Fi credentials
    esp_err_t ret = nvs_flash_init();
    // Handle cases where NVS is full or has an incompatible version
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase NVS to reset it
        ret = nvs_flash_init(); // Re-initialize NVS
    }
    ESP_ERROR_CHECK(ret); // Check for errors during NVS initialization

    // Initialize the network interface for TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    // Create the default event loop to handle system events
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create a default Wi-Fi station interface
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register the event handler for all Wi-Fi events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    // Register the event handler for IP event (specifically when IP is assigned)
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Configure Wi-Fi credentials for station mode
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,     // Set the SSID from the defined macro
            .password = WIFI_PASS, // Set the password from the defined macro
        },
    };

    // Set Wi-Fi mode to station (client) mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Apply the Wi-Fi configuration (SSID and password)
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    // Start the Wi-Fi module
    ESP_ERROR_CHECK(esp_wifi_start());

    // Delay for 5 seconds to allow time for the connection to establish
    vTaskDelay(pdMS_TO_TICKS(5000));
}
