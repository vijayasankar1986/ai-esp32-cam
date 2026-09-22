// Copy to secrets.h and fill in your own values.
// secrets.h is gitignored. Never commit Wi-Fi credentials to a public repository.
#pragma once

#define WIFI_SSID     "Airtel_Vijay"
#define WIFI_PASSWORD "Vijay@123"

// Optional fixed address so the camera IP never moves. Leave HOST_IP empty
// to use DHCP, then read the address the camera prints over serial.
#define HOST_IP       ""
#define GATEWAY_IP    "192.168.1.1"
#define SUBNET_MASK   "255.255.255.0"
