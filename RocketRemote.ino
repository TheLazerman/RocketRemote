#include "LV_Helper.h"
#include "ui.h"
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <array>
#include <deque>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float thrust;
  float pressure;
  bool armed;
  long timestamp;
} struct_message;
struct_message standData;

typedef struct struct_command {
  bool armed;
  bool fire;
  bool reset;
} struct_command;
struct_command testStandCommands;
esp_now_peer_info_t peerInfo;

// MAC Address of the Teststand ESP Device
uint8_t broadcastAddress[] = { 0xb4, 0xe6, 0x2d, 0x69, 0xd6, 0xa0 };

// IO Definitions
#define safeLED 1
#define armedLED 2
#define armingButton 3
#define ignitionButton 10



long lastMillis = 0;
long lastMsg = 0;
long debounce = 0;

bool armed = false;
bool eventInProgress = false;
bool eventOccurred = false;
bool eventComplete = false;
bool standConnected = false;

float maxThrust = 0;
float lastThrust = 0;
float avgThrust = 0;
float totalImpulse = 0;
float rateOfChange = 0;

std::vector<float> thrustCurve;

static lv_chart_series_t * ser1;
static lv_chart_series_t * ser2;

// Fine Tuning variables
float eventStartThreshold = 0.5;
float eventStopThreshold = 0.5 ;
int windowSize = 5; 
std::deque<float> window;


// Callback when data is sent to the teststand 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  standConnected = (status == ESP_NOW_SEND_SUCCESS ? true : false);
  if (!eventComplete){
    if (standConnected){
      if (lv_scr_act() != ui_LiveDataScreen){
        lv_disp_load_scr(ui_LiveDataScreen);
      }
    }else{
      lv_disp_load_scr(ui_DisconnectScreen);
    }

  } 
}


void toggle_arm() {
  if (armed) {
    testStandCommands.armed = false;
  } else {
    testStandCommands.armed = true;
  }
  esp_err_t comResult = esp_now_send(broadcastAddress, (uint8_t *)&testStandCommands, sizeof(testStandCommands));
}


void fire() {
  if (armed){
    testStandCommands.fire = true;
    esp_err_t fireResult = esp_now_send(broadcastAddress, (uint8_t *)&testStandCommands, sizeof(testStandCommands));
  }
}


void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("Starting UI");
    
  lv_helper();
  ui_init();

  ser1 = lv_chart_add_series(ui_ThrustChart1, lv_color_hex(0xE60019), LV_CHART_AXIS_PRIMARY_Y);
  ser2 = lv_chart_add_series(ui_ThrustChartFinal, lv_color_hex(0x00FF13), LV_CHART_AXIS_PRIMARY_Y);

  analogWrite(38,255); // ?? What is this line doing? 

  pinMode(armingButton, INPUT_PULLUP);
  pinMode(ignitionButton, INPUT_PULLUP);
  pinMode(armedLED, OUTPUT);
  pinMode(safeLED, OUTPUT);
     //Turn on display power
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info       
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);

}


double GramsToNewtons(double grams){
  return (grams/1000)*9.81;
}


// Function to calculate the instantaneous rate of change
float calculateInstantaneousRateOfChange(const std::deque<float>& data, int currentIndex) {
  if (currentIndex < 1 || currentIndex > data.size() || data.empty()) {
    return 0.0; 
  }

  float rateOfChange = (data[currentIndex] - data[currentIndex - 1]); 
  return rateOfChange;
}


// Function to detect event start and stop based on rate of change
bool detectEvent(float rateOfChange, float eventStartThreshold, float eventStopThreshold) {
  // Check if the rate of change exceeds the start threshold (event begins)
  if (std::abs(rateOfChange) > eventStartThreshold) {
    eventOccurred = true; // An event has infact, occurred
    return true; // Event started
  }
  // Check if the rate of change falls below the stop threshold (event ends)
  else if (std::abs(rateOfChange) < eventStopThreshold) { 
    return false; // Event stopped 
  } 
  // If neither condition is met, maintain the previous state
  else {
    static bool eventHappeningNow = false; // Keep track of the event state
    return eventHappeningNow; 
  }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  lastMsg = millis();
  memcpy(&standData, incomingData, sizeof(standData));
  armed = standData.armed;
  
  // Update states
  if (armed){
    digitalWrite(armedLED, HIGH);
    digitalWrite(safeLED, LOW);
  } else {
    digitalWrite(armedLED, LOW);
    digitalWrite(safeLED, HIGH);
  }

  // We're done here
  if (eventComplete){
    displayResults();
    return;
  } 

  // TODO
  // Move this to the teststand itself 
  standData.thrust = GramsToNewtons(standData.thrust);

  // Capture the peak thrust value
  if (standData.thrust > maxThrust){
    maxThrust = standData.thrust;
    // Update the y axis range to fit the new max value
    lv_chart_set_range(ui_ThrustChart1, LV_CHART_AXIS_PRIMARY_Y, 0, maxThrust + 0.5 );
  }


  // Do not capture negative values 
  if (standData.thrust >= 0 ){
    thrustCurve.push_back(standData.thrust);
  }

  // Calculate the moving average of the window
  double windowSum = std::accumulate(window.begin(), window.end(), 0.0);
  double movingAverage = window.size() > 0 ? windowSum / window.size() : 0.0; // Handle empty window

  // Calculate rate of change using the PREVIOUS moving average values
  if (window.size() > 1) {
    rateOfChange = calculateInstantaneousRateOfChange(window, window.size()-2);
  } else {
    rateOfChange = 0.0; // Initialize with 0 for the first iteration
  }

  // Push the new moving average into the window for the NEXT calculation
  window.push_back(movingAverage);
  if (window.size() > windowSize) {
    window.pop_front();
  }

  // rateOfChange = calculateInstantaneousRateOfChange(thrustCurve, thrustCurve.size());
  eventInProgress = detectEvent(rateOfChange, eventStartThreshold, eventStopThreshold);

  if (!eventComplete && eventOccurred && !eventInProgress){
    eventComplete = true;
    return;
  }


  if (!eventComplete && eventOccurred){

    String max_string = String(maxThrust);
    _ui_label_set_property(ui_MaxThrustValue, _UI_LABEL_PROPERTY_TEXT, String(max_string + " n").c_str());
    lv_chart_set_next_value(ui_ThrustChart1, ser1, standData.thrust);
    String avg_string = String(avgThrust);
    String impulse_string = String (totalImpulse);
    
    avgThrust = std::accumulate(thrustCurve.begin(), thrustCurve.end(), 0.0) / thrustCurve.size();
    totalImpulse = std::accumulate(thrustCurve.begin(), thrustCurve.end(), 0.0) * 0.1;

    _ui_label_set_property(ui_AvgThrustValue, _UI_LABEL_PROPERTY_TEXT, String(avg_string + " n").c_str());
    _ui_label_set_property(ui_ImpulseValue, _UI_LABEL_PROPERTY_TEXT, String(impulse_string + " N-s").c_str());
  }
}


void displayResults(){
  static bool resultsDisplayed = false;
  if (resultsDisplayed){
    return;
  }
  resultsDisplayed = true;
  // lv_scr_load_anim(ui_ResultsScreen, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, false);
  lv_disp_load_scr(ui_ResultsScreen);
  lv_chart_set_range(ui_ThrustChartFinal, LV_CHART_AXIS_PRIMARY_Y, 0, maxThrust + 0.5 );
  short int arr[thrustCurve.size()];
  transform(thrustCurve.begin(),thrustCurve.end(),arr,[](const int & x){return x;});
  lv_chart_set_ext_y_array(ui_ThrustChartFinal, ser2, arr);
  lv_chart_refresh(ui_ThrustChartFinal);
  String max_string = String(maxThrust);
  String avg_string = String(avgThrust);
  String impulse_string = String(totalImpulse);
  String duration_string = String(float (thrustCurve.size() / 100));
  _ui_label_set_property(ui_MaxThrustValueFinal, _UI_LABEL_PROPERTY_TEXT, String(max_string + " N").c_str());
  _ui_label_set_property(ui_AvgThrustValueFinal, _UI_LABEL_PROPERTY_TEXT, String(avg_string + " N").c_str());
  _ui_label_set_property(ui_ImpulseValueFinal, _UI_LABEL_PROPERTY_TEXT, String(impulse_string + " N-sec").c_str());
  _ui_label_set_property(ui_BurnDurationValue, _UI_LABEL_PROPERTY_TEXT, String(duration_string + " sec").c_str());
  _ui_label_set_property(ui_MotorClassificationValue, _UI_LABEL_PROPERTY_TEXT, String("E56").c_str());

}

void loop() {
  Serial.println("eventComplete:" + String(eventComplete) + "\nEvent:" + String(eventInProgress) + "\nOccur:" + String(eventOccurred) + "\nRate:" + String(std::abs(rateOfChange)) + "\nThrust:" + String(standData.thrust));
  if (millis() - debounce > 250 ){
    if (digitalRead(armingButton) == 0) {
      toggle_arm();
      debounce = millis();
    }
    if (digitalRead(ignitionButton) == 0) {
      fire();
      debounce = millis();
    }
  }
  lv_task_handler();
  // delay(5);
}

