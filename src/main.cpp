/* ========================================================================== *
 * Name:       MAV_Commander
 * -------------------------------------------------------------------------- 
    Function:   Das Programm Steuert eine Drohne über MAVLink 
    -------------------------------------------------------------------------
    Version 1.0: 
    * Anzeige auf dem kleinen Display:
      - Flugzustand
      - Anzeige ARMED /NOT ARMED
    * Anzeige auf dem grossen Display:
      - Anzeige der Nachrichten

    -------------------------------------------------------------------------
    Version 1.1:
    * Schalten von 4 Sonderfunktionen auf 4 RC-Kanälen

 * ========================================================================== */
#define DEBUGGING
#define SW_Version "00.00.003"

/* =========================================================================== */
/* -------------------------- Basis Libs einbinden --------------------------- */
/* =========================================================================== */
#include <Arduino.h>

/* =========================================================================== */
/* ---------------------------- LCD Libs einbinden --------------------------- */
/* =========================================================================== */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* =========================================================================== */
/* --------------------------- Mavlink Libs einbinden ------------------------ */
/* =========================================================================== */
#include <common/mavlink.h>
#include <common/common.h>

/* ========================================================================== *
 *       2 x 16 Status Display
 * ========================================================================== */
#define LCD1COLUMS           16   //LCD columns
#define LCD1ROWS             2    //LCD LCD1ROWS
#define LCD1_SPACE_SYMBOL 0x20    //space symbol from LCD ROM, see p.9 of GDM2004D datasheet


/* ========================================================================== *
 *       4 x 20 Menue Display
 * ========================================================================== */
#define LCD2COLUMS        20   //LCD columns
#define LCD2ROWS          4    //LCD LCD1ROWS
#define LCD2_SPACE_SYMBOL 0x20 //space symbol from LCD ROM, see p.9 of GDM2004D datasheet

/* ========================================================================== */
//      Local MAVLink Device (Ground Station)
/* ========================================================================== */
int      local_system_id      = 240;                      // ID 20 for this airplane. 1 PX, 255 ground station
int      local_component_id   = 1;                        // The component sending the message
int      local_mav_type       = MAV_TYPE_GCS;             // This system is Groundstation
uint8_t  local_autopilot_type = MAV_AUTOPILOT_INVALID;    // Groundstation
uint8_t  local_system_mode    = MAV_MODE_PREFLIGHT;       // Booting up
uint32_t local_custom_mode    = 0;                        // Custom mode, can be defined by user/adopter
uint8_t  local_system_state   = MAV_STATE_STANDBY;        // System ready for flight

unsigned long heartbeat_timer    = millis();
unsigned long heartbeat_interval = 2000;
unsigned long request_timer      = millis();
unsigned long request_intervall  = 300000;

/* ========================================================================== */
//      Remote MAVLink Device (Drone)
/* ========================================================================== */
uint8_t target_system    = 1;
uint8_t target_component = 1;

mavlink_message_t mav_msg;
uint8_t ArmedStatusOld = 2;
uint8_t ArmedStatusNew = 2;

/* ========================================================================== */
//      Funktionsprototypen
/* ========================================================================== */
void dispatch_MavMessage();
void display_DroneStatusOnLcd();
void display_DroneMessagesOnLcd(mavlink_message_t* mav_msg);

/* ========================================================================== */
//      Objekte definieren
/* ========================================================================== */
LiquidCrystal_I2C lcd1(PCF8574A_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);
LiquidCrystal_I2C lcd2(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

/* =========================================================================== */
/* --------------------------------------------------------------------------- */
/* ------------------------ Setup wird einmal durchlaufen -------------------- */
/* --------------------------------------------------------------------------- */
/* =========================================================================== */
void setup() {
  /* ========================================================================= *
   *    2 x 16 Status Display
   * ========================================================================= */
  while (lcd1.begin(LCD1COLUMS, LCD1ROWS, LCD_5x8DOTS) != 1)        //LCD1COLUMS, LCD1ROWS, characters size
  {
    Serial.println("PCF8574 (LCD1) is not connected or lcd1 pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal.");
    delay(5000);
  }
  lcd1.print("LCD2 is OK ...");    //(F()) saves string to flash & keeps dynamic memory free
  lcd1.clear();

  /* ========================================================================= *
   *    4 x 20 Menue Display
   * ========================================================================= */
  while (lcd2.begin(LCD2COLUMS, LCD2ROWS, LCD_5x8DOTS) != 1)       //LCD2COLUMS, LCD2ROWS, characters size
  {
    Serial.println("PCF8574 (LCD2) is not connected or lcd1 pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal.");
    delay(5000);
  }
  lcd2.print("LCD2 is OK ...");    //(F()) saves string to flash & keeps dynamic memory free
  lcd2.clear();

  /* ========================================================================= *
  //    Serielles MAVLink Interface starten
   * ========================================================================= */
  Serial1.begin(115200);

  // ------------------------------------------------------------------------- *
  //    Seriellen Monitor starten
  // ------------------------------------------------------------------------- *
  Serial.begin(9600);
  Serial.println("\n");
  Serial.println("---------------------------");
  Serial.println("  Start MAV Controller ...");
  Serial.println("---------------------------");
}

/* =========================================================================== */
/* --------------------------------------------------------------------------- */
/* ------------------------------ Hauptschleife ------------------------------ */
/* --------------------------------------------------------------------------- */
/* =========================================================================== */
void loop() {
  dispatch_MavMessage();
}

/* =========================================================================== */
/* --------------------------------------------------------------------------- */
/* -------------------------------- Funtionen -------------------------------- */
/* --------------------------------------------------------------------------- */
/* =========================================================================== */

/* =========================================================================== *
 *                          dispatch_MavMessage                               *
 * --------------------------------------------------------------------------- *

 * =========================================================================== */
void dispatch_MavMessage(){
  // mavlink_message_t mav_msg;
  mavlink_status_t mav_msg_status;

  /* ------------------------------------------------------------------------- *
         Prüfen, ob neues Zeichen empfangen wurde.
   * ------------------------------------------------------------------------- */
  while (Serial1.available() > 0) {
    uint8_t receive_char = Serial1.read();
    /* ----------------------------------------------------------------------- *
     *     Try to get a new message
     * ----------------------------------------------------------------------- */
    if (mavlink_parse_char(MAVLINK_COMM_0, 
                           receive_char, 
                           &mav_msg, 
                           &mav_msg_status)) {
      
      switch (mav_msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          display_DroneStatusOnLcd(); 
      }
    }
  }
}
/* =========================================================================== *
 *                        display_DroneStatusOnLcd                             *
 * --------------------------------------------------------------------------- *

 * =========================================================================== */
void display_DroneStatusOnLcd(){
  Serial.println("Hard Beat");
  mavlink_heartbeat_t heard_beat;

  mavlink_msg_heartbeat_decode(&mav_msg, &heard_beat);

  /* ------------------------------------------------------------------------- *
        Anzeige auf LCD 1: Armed oder Disarmed
   * ------------------------------------------------------------------------- */
  heard_beat.base_mode == 209 ? ArmedStatusNew = 1 : ArmedStatusNew = 0;
  if ((ArmedStatusNew == 1) && (ArmedStatusOld != 1)){
    lcd1.setCursor(0, 0);
    lcd1.print("---- ARMED  ----");
    ArmedStatusOld = 1;
  }
  if ((ArmedStatusNew == 0) && (ArmedStatusOld != 0)){
    lcd1.setCursor(0, 0);
    lcd1.print("----DISARMED----");
    ArmedStatusOld = 0;
  }

  //Serial.print("State        : "); Serial.println(heard_beat.base_mode == 209 ? "Armed" : "Disarmed");
  Serial.print("Mode         : ");
  
  switch(heard_beat.custom_mode) {
    case 0:
      Serial.println("Manual");
    break;
    case 1:
      Serial.println("Circle");
    break;
    case 2:
      Serial.println("Stabilize");
    break;
    case 3:
      Serial.println("Training");
    break;
    case 5:
      Serial.println("FBWA");
    break;
    case 6:
      Serial.println("FBWB");
    break;
    case 7:
      Serial.println("Cruise");
    break;
    case 8:
      Serial.println("AUTOTUNE");
    break;
    case 10:
      Serial.println("Auto");
    break;
    case 11:
      Serial.println("RTL");
    break;
    case 12:
      Serial.println("Loiter");
    break;
    case 13:
      Serial.println("Take Off");
    break;
    case 14:
      Serial.println("Avoid ADSB");
    break;
    case 15:
      Serial.println("Guided");
    break;
    case 17:
      Serial.println("QStabilize");
    break;
    case 18:
      Serial.println("QHover");
    break;
    case 19:
      Serial.println("QLoiter");
    break;
    case 20:
      Serial.println("QLand");
    break;
    case 21:
      Serial.println("QRTL");
    break;
    case 22:
      Serial.println("QAutoTune");
    break;
    case 23:
      Serial.println("QAcro");
    break;
    case 24:
      Serial.println("Thermal");
    break;
    case 25:
      Serial.println("Loiter to QLand");
    break;
    default:
      Serial.print("Mode "); Serial.print(heard_beat.custom_mode); Serial.println(" not known");
    break;
  }

}


/* =========================================================================== *
 *                        display_DroneMessagesOnLcd                           *
 * --------------------------------------------------------------------------- *

 * =========================================================================== */
void display_DroneMessagesOnLcd(mavlink_message_t* mav_msg){

}
