# FSD-CANbus-tool
FSD CANbus tool

/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "memory"
#include <SPI.h>
#include <mcp2515.h>


#define LEGACY LegacyHandler
#define HW3 HW3Handler
#define HW4 HW4Handler //HW4 since Version 2026.2.3 uses FSDV14, before that compile for HW3, even for HW4 vehicles.


#define HW HW3  //for what car to compile

bool enablePrint = true;


#define LED_PIN PIN_LED                // onboard red LED (GPIO13)
#define CAN_CS PIN_CAN_CS              // GPIO19 on this Feather
#define CAN_INT_PIN PIN_CAN_INTERRUPT  // GPIO22 (unused here; polling)
#define CAN_STBY PIN_CAN_STANDBY       // GPIO16
#define CAN_RESET PIN_CAN_RESET        // GPIO18

std::unique_ptr<MCP2515> mcp;

struct CarManagerBase {
  int speedProfile = 1;
  bool FSDEnabled = false;
  virtual void handelMessage(can_frame& frame);
};

inline uint8_t readMuxID(const can_frame& frame) {
  return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame& frame) {
  return (frame.data[4] >> 6) & 0x01;
}

inline void setSpeedProfileV12V13(can_frame& frame, int profile) {
  frame.data[6] &= ~0x06;
  frame.data[6] |= (profile << 1);
}

inline void setBit(can_frame& frame, int bit, bool value) {
  // Determine which byte and which bit within that byte
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  // Set the desired bit
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}


struct LegacyHandler : public CarManagerBase {
  virtual void handelMessage(can_frame& frame) override {
    if (frame.can_id == 1006) {
      auto index = readMuxID(frame);
      if (index == 0 && isFSDSelectedInUI(frame)) {
        auto off = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
        switch (off) {
          case 2: speedProfile = 2; break;
          case 1: speedProfile = 1; break;
          case 0: speedProfile = 0; break;
          default: break;
        }
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        mcp->sendMessage(&frame);
      }
      if (index == 1) {
        setBit(frame, 19, false);
        mcp->sendMessage(&frame);
      }
      if (index == 0 && enablePrint) {
        Serial.printf("LegacyHandler: FSD: %d, Profile: %d\n", FSDEnabled, speedProfile);
      }
    }
  }
};

struct HW3Handler : public CarManagerBase {
  int speedOffset = 0;
  virtual void handelMessage(can_frame& frame) override {
    if (frame.can_id == 1016) {
      uint8_t followDistance = (frame.data[5] & 0b11100000) >> 5;
      switch (followDistance) {
        case 1:
          speedProfile = 2;
          break;
        case 2:
          speedProfile = 1;
          break;
        case 3:
          speedProfile = 0;
          break;
        default:
          break;
      }
      return;
    }
    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
      auto FSDEnabled = isFSDSelectedInUI(frame);
      if (index == 0 && FSDEnabled) {
        speedOffset = std::max(std::min(((uint8_t)((frame.data[3] >> 1) & 0x3F) - 30) * 5, 100), 0);
        auto off = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
        switch (off) {
          case 2: speedProfile = 2; break;
          case 1: speedProfile = 1; break;
          case 0: speedProfile = 0; break;
          default: break;
        }
        setBit(frame, 46, true);
        setSpeedProfileV12V13(frame, speedProfile);
        mcp->sendMessage(&frame);
      }
      if (index == 1) {
        setBit(frame, 19, false);
        mcp->sendMessage(&frame);
      }
      if (index == 2 && FSDEnabled) {
        frame.data[0] &= ~(0b11000000);
        frame.data[1] &= ~(0b00111111);
        frame.data[0] |= (speedOffset & 0x03) << 6;
        frame.data[1] |= (speedOffset >> 2);
        mcp->sendMessage(&frame);
      }
      if (index == 0 && enablePrint) {
        Serial.printf("HW3Handler: FSD: %d, Profile: %d, Offset: %d\n", FSDEnabled, speedProfile, speedOffset);
      }
    }
  }
};

struct HW4Handler : public CarManagerBase {
  virtual void handelMessage(can_frame& frame) override {
    if (frame.can_id == 1016) {
      auto fd = (frame.data[5] & 0b11100000) >> 5;
      switch(fd){
        case 1: speedProfile = 3; break;
        case 2: speedProfile = 2; break;
        case 3: speedProfile = 1; break;
        case 4: speedProfile = 0; break;
        case 5: speedProfile = 4; break;
      }
    }
    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
      auto FSDEnabled = isFSDSelectedInUI(frame);
      if (index == 0 && FSDEnabled) {
        setBit(frame, 46, true);
        setBit(frame, 60, true);
        mcp->sendMessage(&frame);
      }
      if (index == 1) {
        setBit(frame, 19, false);
        setBit(frame, 47, true);
        mcp->sendMessage(&frame);
      }
      if(index == 2){
        frame.data[7] &= ~(0x07 << 4);
        frame.data[7] |= (speedProfile & 0x07) << 4;
        mcp->sendMessage(&frame);
      }
      if (index == 0 && enablePrint) {
        Serial.printf("HW4Handler: FSD: %d, profile: %d\n", FSDEnabled, speedProfile);
      }
    }
  }
};


std::unique_ptr<CarManagerBase> handler;


void setup() {
  handler = std::make_unique<HW>();
  delay(1500); 
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1000) {}

  mcp = std::make_unique<MCP2515>(CAN_CS);

  mcp->reset();
  MCP2515::ERROR e = mcp->setBitrate(CAN_500KBPS, MCP_16MHZ);  
  if (e != MCP2515::ERROR_OK) Serial.println("setBitrate failed");
  mcp->setNormalMode();
  Serial.println("MCP25625 ready @ 500k 1");
}


__attribute__((optimize("O3"))) void loop() {
  can_frame frame;
  int r = mcp->readMessage(&frame);
  if (r != MCP2515::ERROR_OK) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  digitalWrite(LED_PIN, LOW);
  handler->handelMessage(frame);
}




OLD Readme below:

CanFeather – Tesla FSD CAN Bus Enabler


Why is this public? Some sellers charge up to 500 € for a solution like this. In our opinion, that is massively overpriced. The board costs around 20 €, and even with labor factored in, a fair price is no more than 50 €. This project exists so nobody has to overpay.


📌 Prerequisites

You must have an active FSD package on the vehicle — either purchased or subscribed. This board enables the FSD functionality on the CAN bus level, but the vehicle still needs a valid FSD entitlement from Tesla.
If FSD subscriptions are not available in your region, you can work around this by:

Creating a Tesla account in a region where FSD subscriptions are offered (e.g. Canada).
Transferring the vehicle to that account.
Subscribing to FSD through that account.

This allows you to activate an FSD subscription from anywhere in the world.

🛠️ What It Does

This firmware runs on an Adafruit CAN Bus FeatherWing (MCP25625/MCP2515-based). It intercepts specific CAN bus messages to enable and configure Full Self-Driving (FSD).
🚗 Core Function

Intercepts specific CAN bus messages
Re-transmits them onto the vehicle bus

🧠 FSD Activation Logic

Listens for Autopilot-related CAN frames
Checks if "Traffic Light and Stop Sign Control" is enabled in the Autopilot settings Uses this setting as a trigger for Full Self-Driving (FSD)
Adjusts the required bits in the CAN message to activate FSD

⚙️ Additional Behavior

Reads the follow-distance stalk setting
Maps it dynamically to a speed profile

Supported Hardware Variants

Select your hardware in CanFeather.ino via the #define HW directive:



Define
Target
Listens on CAN IDs
Notes




LEGACY
HW3 Retrofit
1006
Sets FSD enable bit and speed profile


HW3
HW3 vehicles
1016, 1021
Adds speed-offset control via follow-distance


HW4
HW4 vehicles
1016, 1021
Extended speed-profile range (5 levels)




Note: HW4 vehicles on firmware older than 2026.2.3 do not use FSDV14. If your vehicle is on an earlier firmware version, compile with HW3 even if your vehicle has HW4 hardware.

How to Determine Your Hardware Variant



Legacy — Your vehicle has a portrait-oriented center screen and HW3. This applies to older Model S and Model X vehicles retrofitted with HW3.

HW3 — Your vehicle has a landscape-oriented center screen and HW3. You can check your hardware version under Controls → Software → Additional Vehicle Information on the vehicle's touchscreen.

HW4 — Same as above, but the Additional Vehicle Information screen shows HW4.

Key Behaviour



FSD enable bit is set when "Traffic Light and Stop Sign Control" is enabled in the vehicle's Autopilot settings.

Speed profile is derived from the scroll-wheel offset or follow-distance setting.

Nag suppression — clears the hands-on-wheel nag bit.
Debug output is printed over Serial at 115200 baud when enablePrint is true.

Hardware Requirements


Adafruit Feather M4 CAN (or compatible board with MCP25625/MCP2515)
The board must expose these pins (defined at the top of the sketch):


PIN_CAN_CS — SPI chip-select for the MCP2515

PIN_CAN_INTERRUPT — interrupt pin (unused; polling mode)

PIN_CAN_STANDBY — CAN transceiver standby control

PIN_CAN_RESET — MCP2515 hardware reset


CAN bus connection to the vehicle (500 kbit/s)

Installation

1. Install the Arduino IDE

Download from https://www.arduino.cc/en/software.
2. Add the Adafruit Board Package


Open File → Preferences.
In Additional Board Manager URLs, add:

https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json



Go to Tools → Board → Boards Manager, search for Raspberry PI Pico/RP2040 (or the appropriate family for your Feather), and install it.
Select Adafruit Feather RP2040 CAN as the Board.

3. Install Required Libraries

Install the following library via Sketch → Include Library → Manage Libraries… or the Arduino Library Manager:


MCP2515 by autowp — CAN controller driver (mcp2515.h)

4. Select Your Hardware Target

Near the top of CanFeather.ino, change the HW define to match your vehicle:

#define HW HW3  // Change to LEGACY, HW3, or HW4


5. Upload


Connect the Feather via USB.
Select the correct board and port under Tools.
Click Upload.

6. Wiring

The recommended connection point is the X179 connector:



Pin
Signal




13
CAN-H


14
CAN-L



Connect the Feather's CAN-H and CAN-L lines to pins 13 and 14 on the X179 connector.
The recommended connection point for legacy Model 3 (2020 and earlier) is the X652 connector if the vehicle is not equipped with the X179 port (varies depending on production date):



Pin
Signal




1
CAN-H


2
CAN-L



Connect the Feather's CAN-H and CAN-L lines to pins 1 and 2 on the X652 connector.
Important: Cut the onboard 120 Ω termination resistor on the Feather CAN board. The vehicle's CAN bus already has its own termination, and adding a second resistor will cause communication errors.
Speed Profiles

The speed profile controls how aggressively the vehicle drives under FSD. It is configured differently depending on the hardware variant:
Legacy (HW3 Retrofit)

Because the Legacy variant transmits follow distance differently, it uses a speed offset value (in km/h) to select the profile:



Speed Offset (km/h)
Profile




28
Chill


29
Normal


30
Hurry



HW3 & HW4 Profiles




Distance
Profile (HW3)
Profile (HW4)




2

⚡ Hurry

🔥 Max


3

🟢 Normal

⚡ Hurry


4

❄️ Chill

🟢 Normal


5
—

❄️ Chill


6
—

🐢 Sloth



Serial Monitor

Open the Serial Monitor at 115200 baud to see live debug output showing FSD state and the active speed profile. Disable logging by setting enablePrint = false.
Disclaimer

Use this project at your own risk. Modifying CAN bus messages on a vehicle can lead to unexpected or dangerous behavior. The authors accept no responsibility for any damage to your vehicle, injury, or legal consequences resulting from the use of this software. This project may void your vehicle warranty and may not comply with road safety regulations in your jurisdiction. Always keep your hands on the wheel and stay attentive while driving.
License

This project is licensed under the GNU General Public License v3.0 — see the GPL-3.0 License for details.
