//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <zenoh-pico.h>

#if Z_FEATURE_PUBLICATION == 1
// Client mode values (comment/uncomment as needed)
#define MODE "client"
#define DEFAULT_LOCATOR "tcp/10.0.0.228:7447"  // Default locator
// Peer mode values (comment/uncomment as needed)
// #define MODE "peer"
// #define DEFAULT_LOCATOR "udp/224.0.0.225:7447#iface=en0"

#define KEYEXPR "demo/example/zenoh-pico-pub"
#define VALUE "[ARDUINO]{ESP32} Publication from Zenoh-Pico!"

z_owned_session_t s;
z_owned_publisher_t pub;
static int idx = 0;

// WiFiManager instance
WiFiManager wm;

// Custom parameter for Zenoh locator
char zenoh_locator[100] = DEFAULT_LOCATOR;

void setup() {
    // Initialize Serial for debug
    Serial.begin(115200);
    while (!Serial) {
        delay(1000);
    }

    // WiFiManager setup
    Serial.println("Starting WiFiManager...");

    // Reset settings for testing (uncomment if needed)
    // wm.resetSettings();

    // Create custom parameter for Zenoh locator
    WiFiManagerParameter custom_locator("locator", "Zenoh Locator", zenoh_locator, 100);

    // Add the custom parameter to WiFiManager
    wm.addParameter(&custom_locator);

    // Set WiFi mode
    WiFi.mode(WIFI_STA);

    // Automatically connect using saved credentials or start config portal
    if (!wm.autoConnect("ZenohESP32", "zenoh123")) {
        Serial.println("Failed to connect to WiFi");
        delay(3000);
        ESP.restart();
        delay(5000);
    }

    // Get the custom parameter value
    strcpy(zenoh_locator, custom_locator.getValue());

    Serial.println("Connected to WiFi successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Zenoh Locator: ");
    Serial.println(zenoh_locator);

    // Initialize Zenoh Session and other parameters
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
    if (strlen(zenoh_locator) > 0) {
        if (strcmp(MODE, "client") == 0) {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, zenoh_locator);
        } else {
            zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY, zenoh_locator);
        }
    }

    // Open Zenoh session with retry logic
    Serial.print("Opening Zenoh Session...");
    int zenoh_retry_count = 0;
    const int max_zenoh_retries = 10;
    bool zenoh_connected = false;

    while (zenoh_retry_count < max_zenoh_retries && !zenoh_connected) {
        if (z_open(&s, z_config_move(&config), NULL) >= 0) {
            zenoh_connected = true;
            Serial.println("OK");
        } else {
            zenoh_retry_count++;
            Serial.print(".");
            if (zenoh_retry_count < max_zenoh_retries) {
                // Need to recreate config for next attempt
                z_config_default(&config);
                zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_MODE_KEY, MODE);
                if (strlen(zenoh_locator) > 0) {
                    if (strcmp(MODE, "client") == 0) {
                        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_CONNECT_KEY, zenoh_locator);
                    } else {
                        zp_config_insert(z_config_loan_mut(&config), Z_CONFIG_LISTEN_KEY, zenoh_locator);
                    }
                }
                delay(1000);
            }
        }
    }

    if (!zenoh_connected) {
        Serial.println("\nFailed to connect to Zenoh router after 10 attempts!");
        Serial.println("Resetting WiFi settings and restarting...");
        wm.resetSettings();
        delay(1000);
        ESP.restart();
    }

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan_mut(&s), NULL) < 0 || zp_start_lease_task(z_session_loan_mut(&s), NULL) < 0) {
        Serial.println("Unable to start read and lease tasks\n");
        z_session_drop(z_session_move(&s));
        while (1) {
            ;
        }
    }

    // Declare Zenoh publisher
    Serial.print("Declaring publisher for ");
    Serial.print(KEYEXPR);
    Serial.println("...");
    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str_unchecked(&ke, KEYEXPR);
    if (z_declare_publisher(z_session_loan(&s), &pub, z_view_keyexpr_loan(&ke), NULL) < 0) {
        Serial.println("Unable to declare publisher for key expression!");
        while (1) {
            ;
        }
    }
    Serial.println("OK");
    Serial.println("Zenoh setup finished!");

    delay(300);
}

void loop() {
    delay(1000);
    char buf[256];
    sprintf(buf, "[%4d] %s", idx++, VALUE);

    Serial.print("Writing Data ('");
    Serial.print(KEYEXPR);
    Serial.print("': '");
    Serial.print(buf);
    Serial.println("')");

    // Create payload
    z_owned_bytes_t payload;
    z_bytes_copy_from_str(&payload, buf);

    if (z_publisher_put(z_publisher_loan(&pub), z_bytes_move(&payload), NULL) < 0) {
        Serial.println("Error while publishing data");
    }
}
#else
void setup() {
    Serial.println("ERROR: Zenoh pico was compiled without Z_FEATURE_PUBLICATION but this example requires it.");
    return;
}
void loop() {}
#endif
