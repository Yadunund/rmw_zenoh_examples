// Copyright 2025 Yadunund Vijay.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>

#include <picoros.h>
#include <picoserdes.h>
#include <picoparams.h>

#if Z_FEATURE_PUBLICATION == 1
// Client mode values (comment/uncomment as needed)
#define MODE "client"
#define DEFAULT_LOCATOR "tcp/10.0.0.228:7447"  // Default locator
// Peer mode values (comment/uncomment as needed)
// #define MODE "peer"
// #define DEFAULT_LOCATOR "udp/224.0.0.225:7447#iface=en0"

extern int picoros_parse_args(int argc, char **argv, picoros_interface_t* ifx);

#define KEYEXPR "demo/example/zenoh-pico-pub"
#define VALUE "[ARDUINO]{ESP32} Publication from Zenoh-Pico!"

z_owned_session_t s;
z_owned_publisher_t pub;
static int idx = 0;

// WiFiManager instance
WiFiManager wm;

// Custom parameter for Zenoh locator
char zenoh_locator[100] = DEFAULT_LOCATOR;


// Example Publisher
picoros_publisher_t pub_log = {
    .topic = {
        .name = "chatter",
        .type = ROSTYPE_NAME(ros_String),
        .rihs_hash = ROSTYPE_HASH(ros_String),
    },
};

// Example node
picoros_node_t node = {
    .name = "talker",
};

// Buffer for publication, used from this thread
uint8_t pub_buf[1024];

void publish_log(){
    printf("Publishing log...\n");
    char* msg = const_cast<char*>("Hello from Pico-ROS!");
    size_t len = ps_serialize(pub_buf, &msg, 1020);
    picoros_publish(&pub_log, pub_buf, len);
}

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
    picoros_interface_t ifx = {
        .mode = const_cast<char*>(MODE),
        .locator = zenoh_locator,
    };
    // Since we're on Arduino (no argc/argv), we skip picoros_parse_args
    // and use the ifx structure directly with the WiFiManager configured locator
    int ret = 0; // Assume success since we set the interface directly

    if(ret != 0){
        Serial.println("\nFailed to connect to Zenoh router after 10 attempts!");
        Serial.println("Resetting WiFi settings and restarting...");
        wm.resetSettings();
        delay(1000);
        ESP.restart();
    }

    printf("Starting pico-ros interface %s %s\n", ifx.mode, ifx.locator );
    while (picoros_interface_init(&ifx) == PICOROS_NOT_READY){
        printf("Waiting RMW init...\n");
        z_sleep_s(1);
    }

    // Declare Zenoh publisher
    printf("Starting Pico-ROS node %s domain:%d\n", node.name, node.domain_id);
    picoros_node_init(&node);

    printf("Declaring publisher on %s\n", pub_log.topic.name);
    picoros_publisher_declare(&node, &pub_log);
    Serial.println("Zenoh setup finished!");

    delay(300);
}

void loop() {
    publish_log();
    z_sleep_s(1);
}
#else
void setup() {
    Serial.println("ERROR: Zenoh pico was compiled without Z_FEATURE_PUBLICATION but this example requires it.");
    return;
}
void loop() {}
#endif
