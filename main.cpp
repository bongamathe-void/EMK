#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* apSSID = "BamzeeESP";
const char* apPassword = "Password1";

enum CommState { IDLE, QUERY_SENT, WAITING_RESPONSE, RESPONSE_RECEIVED };
CommState commState = IDLE;
unsigned long responseStartTime;
const unsigned long responseTimeout = 7000; // Example timeout in milliseconds
String picResponse = "";

String Main_Slogan = "Programmed Slogan";
String CalibrationStatus = "Red";
String CalStatuses[] = {"Red", "Green", "Blue", "Done"};
String RaceStatus = "Idle";
String RaceStatuses[] = {"Idle", "Driving"};
String RaceColor = "Red";
String RaceColors[] = {"Red", "Green", "Blue", "Black"};
String currentQueryCode = "";
String prevQueryCode = "";

String S1color = "rgb(13, 0, 0)";
String S2color = "rgb(13, 0, 0)";
String S3color = "rgb(13, 0, 0)";
String S4color = "rgb(13, 0, 0)";
String S5color = "rgb(13, 0, 0)";

ESP8266WebServer server(80); 

const char MainHead[] = "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                        "<link rel=\"icon\" href=\"data:,\">"
                        "<style>html { font-family: monospace; display: inline-block; margin: 0px auto; text-align: center;}"
                        ".button { background-color: #6598eb; border: none; color: white; padding: 16px 28px;"
                        "text-decoration: none; font-size: 16px; margin: 2px; cursor: pointer;}"
                        "body { background-image: linear-gradient(to right, rgb(252, 252, 251), rgb(118, 12, 120)); }" 
                        "</style></head>"
                        "<header><p><a href=\"/Home\"><button class=\"button\">Home</button></a><br><br></p></header>";

const char homePage[] PROGMEM = "<body>"
                                "<p><a href=\"/SelectColor\"><button class=\"button\">Select Color</button></a><br><br></p>"
                                "<p><a href=\"/Calibration\"><button class=\"button\">Calibration</button></a><br><br></p>"
                                "<p><a href=\"/Race\"><button class=\"button\">Race</button></a><br><br></p>"
                                "<p><a href=\"/Diagnostics\"><button class=\"button\">Diagnostics</button></a><br><br></p>"
                                "<p><a href=\"/Program\"><button class=\"button\">Program</button></a><br><br></p>"
                                "<p><a href=\"/Echo\"><button class=\"button\">Echo</button></a><br><br></p>"
                                "</body>";
const char ColorSelect[] PROGMEM = "<body>"
                                   "<h1>Choose Race Color: </h1>"
                                   "<p><a href=\"/SelectColor/Red\"><button class=\"button\" style=\"background-color:rgb(239, 5, 5);\">Red</button></a><br><br></p>"
                                   "<p><a href=\"/SelectColor/Green\"><button class=\"button\" style=\"background-color:rgb(27, 236, 86);\">Green</button></a><br><br></p>"
                                   "<p><a href=\"/SelectColor/Blue\"><button class=\"button\" style=\"background-color:rgb(5, 21, 239);\">Blue</button></a><br><br></p>"
                                   "<p><a href=\"/SelectColor/Black\"><button class=\"button\" style=\"background-color:rgb(13, 0, 0);\">Black</button></a><br><br></p>"
                                   "</body>";
const char CalibrationPage[] PROGMEM = R"rawliteral(<body>
                                       <h1>Calibration Status : </h1>
                                       <p><a href="/Calibration/Status"><button class="button">Status</button></a><br><br></p>
                                       <div style="width: 200px; height: 200px; background-color: %Color%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white; margin: 0 auto;">
                                       %CalibrationStatus%
                                       </div>
                                       </body>)rawliteral";
const char RacePage[] PROGMEM = R"rawliteral(
                                  <body>
                                  <h2>Racing Status: %RacingStatus% </h2> 
                                  <p><a href="/Race/RaceStatus"><button class="button">Refresh</button></a><br><br></p>
                                  <h2>Race Color: </h2>
                                  <div style="width: 200px; height: 200px; background-color: %Color%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white; margin: 0 auto;">
                                  %RacingColor%
                                  </div>
                                  <p><a href="/Race/RaceColor"><button class="button">Refresh</button></a><br><br></p>
                                  </body>
                                  )rawliteral";
const char ProgramPage[] PROGMEM = R"rawliteral(
                                    <body>
                                    <p><a href="/Program/UpdateSlogan"><button class="button">Request Change</button></a><br><br></p>
                                    <form action="/Program/SendBackSlogan" method="get">
                                    <label for="inputText">Enter Text:</label>
                                    <input type="text" id="inputText" name="inputText"><br><br>
                                    <input type="submit" value="Submit">
                                    </form>
                                    </body>
                                    )rawliteral";

const char DiagnosticsPage[] PROGMEM = R"rawliteral(
                                        <body>
                                        <h2> Sensor data : </h2>
                                        <div style="display: flex; justify-content: center; gap: 10px; margin-bottom: 20px;">
                                          <div style="width: 50px; height: 50px; background-color: %Co1or%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white;">S1</div>
                                          <div style="width: 50px; height: 50px; background-color: %Co2or%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white;">S2</div>
                                          <div style="width: 50px; height: 50px; background-color: %Co3or%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white;">S3</div>
                                          <div style="width: 50px; height: 50px; background-color: %Co4or%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white;">S4</div>
                                          <div style="width: 50px; height: 50px; background-color: %Co5or%; display: flex; align-items: center; justify-content: center; font-size: 20px; border: 2px solid black; color: white;">S5</div>
                                        </div>
                                        <p><a href="/Diagnostics/Sensors"><button class="button">Refresh</button></a><br><br></p>
                                        <p><a href="/Diagnostics/Foward"><button class="button" style="background-color:rgb(109, 109, 126);">Foward</button></a><br><br></p>
                                        <p><a href="/Diagnostics/Left"><button class="button" style="background-color:rgb(109, 109, 126);">Left</button></a>
                                        <a href="/Diagnostics/Right"><button class="button" style="background-color:rgb(109, 109, 126);">Right</button></a>
                                        <br><br></p>
                                        <p><a href="/Diagnostics/Stop"><button class="button" style="background-color:rgb(109, 109, 126);">Stop</button></a><br><br></p>
                                        </body>
                                        )rawliteral";

const char EchoPage[] PROGMEM = R"rawliteral(
                                <body>
                                <p><a href="/Echo"><button class="button">Request Echo</button></a><br><br></p>
                                <form action="/Echo/SendMessage" method="get">
                                <label for="inputText">Enter Text:</label>
                                <input type="text" id="inputText" name="inputText"><br><br>
                                <input type="submit" value="Submit">
                                </form>
                                </body>
                                )rawliteral";

String CurrentPage = "";
//---------------------------------------------------------------------------------------------------------------------------------------------------

//Functions 
void updateResponse(String PICResponse);
void SendPicQuery(String query);
void handleCurrentPage();
void handleHome();
void handleSelectColor();
void handleSCRed();
void handleSCGreen();
void handleSCBlue();
void handleSCBlack();
void handleCalibration();
void handleCalStatus();
void handleRace();
void handleRaceStatus();
void handleRaceColor();
void checkPICResponse();
void handleProgram();
void requestSloganChange();
void transmitSlogan();
void receiveSlogan();
void handleDiagnostics();
void handleSensors();
void handleFoward();
void handleLeft();
void handleRight();
void handleStop();
void handleEcho();
void transmitEchoMessage();
//--------------------------------------------------------------

void setup() { 
  delay(1000);
  Serial.begin(115200);
  //Serial.println();
  //Serial.println("Configuring access point...");
  WiFi.softAP(apSSID, apPassword);
  IPAddress apIP = WiFi.softAPIP();
  //Serial.print("AP IP address: ");
  //Serial.println(apIP);

  delay(1000);

  //Serial.println("Access Point is ready. Connect your device to it.");
  CurrentPage = homePage; // This will change to be dependant on the PIC| later change to a "999" query mode to update currentPage
  server.on("/", handleCurrentPage); // Define web server routes (after AP is set up)
  server.on("/Home", handleHome);
  server.on("/SelectColor", handleSelectColor);
  server.on("/SelectColor/Red", handleSCRed);
  server.on("/SelectColor/Green", handleSCGreen);
  server.on("/SelectColor/Blue", handleSCBlue);
  server.on("/SelectColor/Black", handleSCBlack);
  server.on("/Calibration", handleCalibration);
  server.on("/Calibration/Status", handleCalStatus);
  server.on("/Race", handleRace);
  server.on("/Race/RaceStatus", handleRaceStatus);
  server.on("/Race/RaceColor", handleRaceColor);
  server.on("/Program", handleProgram);
  server.on("/Program/UpdateSlogan", requestSloganChange);
  server.on("/Program/SendBackSlogan", transmitSlogan);
  server.on("/Diagnostics", handleDiagnostics);
  server.on("/Diagnostics/Sensors", handleSensors);
  server.on("/Diagnostics/Foward", handleFoward);
  server.on("/Diagnostics/Left", handleLeft);
  server.on("/Diagnostics/Right", handleRight);
  server.on("/Diagnostics/Stop", handleStop);
  server.on("/Echo", handleEcho);
  server.on("/Echo/SendMessage", transmitEchoMessage);
  server.begin();             // Start the web server (after AP is set up)

  delay(1000);
  //Serial.println("Web server started");
  Serial.print("#512\r\n");
  while(!Serial.available()){
    Main_Slogan = Serial.readStringUntil('\n');
    break;
  }
}

void loop() {
  server.handleClient();
  checkPICResponse();
}
// REQUEST AND RESPONSE CODES :
// "000" home request | "001" home mode
// "999" query mode

// "100" Select color request |  "101" Select color mode
// "110" Choose Red
// "120" Choose Green
// "130" Choose Blue
// "140" Choose Black

// "200" Calibration request  |  "201" Calibration mode
// "202" Calibration status query | "211" Calibrating Red | "221" Calibrating Green | "231" Calibrating Blue | "241" Calibrating Done

// "300" Race request         |  "301" Race mode
// "302" Race Status query    |  "361" Currently Idle   |   "371" Currently Driving
// "303" Race Color query     |  "311" Racing Red         | "321" Racing Green      | "331" Racing Blue      | "341" Racing Black

// "400" Diagnostics request  |  "401" Diagnostics mode
// "410" Query Sensors        |   reply with sensor data
// "420" request foward       |  "401" Diagnostics mode
// "430" request left         |  "401" Diagnostics mode
// "440" request right        |  "401" Diagnostics mode
// "450" request stop         |  "401" Diagnostics mode
// "500" Program request      |  "501" Program mode
// "510" Request slogan change|  "511" Slogan change ack
//  System Sends new slogan   |  "514" Slogan Recieved
// "512" Read slogan          |  
// "700" Echo request         |  "701" Echo mode
// System send sentence       |  sentence echoed back

void updateResponse(String Mode){
  if (Mode == "001"){ // PIC is home mode, display home page
    CurrentPage = F("<div><h1>Team 45 325i</h1><p>");
    CurrentPage += Main_Slogan;
    CurrentPage += F("</p></div>");
    CurrentPage += FPSTR(homePage);
  }
  else if (Mode == "101")
    CurrentPage = String(FPSTR(ColorSelect));
  else if (Mode[0] == '2'){

    if (Mode[1] != '0'){
      CalibrationStatus = CalStatuses[String(Mode[1]).toInt() - 1];
    }
    CurrentPage = String(FPSTR(CalibrationPage));
    CurrentPage.replace("%CalibrationStatus%", CalibrationStatus);

    if (CalibrationStatus == "Red")
      CurrentPage.replace("%Color%", "rgb(239, 5, 5)");
    else if (CalibrationStatus == "Green")
      CurrentPage.replace("%Color%", "rgb(27, 236, 86)");
    else if (CalibrationStatus == "Blue")
      CurrentPage.replace("%Color%", "rgb(5, 21, 239)");
    else
      CurrentPage.replace("%Color%", "rgb(13, 0, 0)");

  }
  else if (Mode[0] == '3'){
    
    if (Mode[1] == '6' || Mode[1] == '7'){
      RaceStatus = RaceStatuses[String(Mode[1]).toInt() - 6];
    }
    else if (Mode[1] != '0'){
      RaceColor = RaceColors[String(Mode[1]).toInt() - 1];
    }

    CurrentPage = String(FPSTR(RacePage));

    CurrentPage.replace("%RacingStatus%", RaceStatus);
    CurrentPage.replace("%RacingColor%", RaceColor);

    if (RaceColor == "Red")
      CurrentPage.replace("%Color%", "rgb(239, 5, 5)");
    else if (RaceColor == "Green")
      CurrentPage.replace("%Color%", "rgb(27, 236, 86)");
    else if (RaceColor == "Blue")
      CurrentPage.replace("%Color%", "rgb(5, 21, 239)");
    else
      CurrentPage.replace("%Color%", "rgb(13, 0, 0)");
  }
  else if (Mode[0] == '4'){
    if (Mode[1] == '0'){
      CurrentPage = String(FPSTR(DiagnosticsPage));

      CurrentPage.replace("%Co1or%", S1color);
      CurrentPage.replace("%Co2or%", S2color);
      CurrentPage.replace("%Co3or%", S3color);
      CurrentPage.replace("%Co4or%", S4color);
      CurrentPage.replace("%Co5or%", S5color);

      if (currentQueryCode == "#420\r\n") {
      CurrentPage += R"rawliteral(<p>Driving forward</p>)rawliteral";
    } else if (currentQueryCode == "#430\r\n") {
      CurrentPage += R"rawliteral(<p>Turning left</p>)rawliteral";
    } else if (currentQueryCode == "#440\r\n") {
      CurrentPage += R"rawliteral(<p>Turning right</p>)rawliteral";
    } else if (currentQueryCode == "#450\r\n") {
      CurrentPage += R"rawliteral(<p>Coming to a stop</p>)rawliteral";
    } 
    }
    else{
      CurrentPage = String(FPSTR(DiagnosticsPage)); 
      CurrentPage += R"rawliteral(<p>Something is definitely wrong...</p>)rawliteral";
    }
  }else if (Mode[0] == '*'){
    CurrentPage = String(FPSTR(DiagnosticsPage));
    
    for (int j = 1; j < 6; j++){
      String color = "";
      if (Mode[j] == '0')
        color = "rgb(13, 0, 0)";
      else if (Mode[j] == '1')
        color = "rgb(239, 5, 5)";
      else if (Mode[j] == '2')
        color = "rgb(27, 236, 86)";
      else if (Mode[j] == '4')
        color = "rgb(5, 21, 239)";
      else 
        color = "rgb(249, 244, 222)";

      if (j == 1)
        S1color = color;
      else if (j == 2)
        S2color = color;
      else if (j == 3)
        S3color = color;
      else if (j == 4)
        S4color = color;
      else
        S5color = color;
    }

    CurrentPage.replace("%Co1or%", S1color);
    CurrentPage.replace("%Co2or%", S2color);
    CurrentPage.replace("%Co3or%", S3color);
    CurrentPage.replace("%Co4or%", S4color);
    CurrentPage.replace("%Co5or%", S5color);
  }
  else if (Mode[0] == '5'){
    if (Mode[1] == '0'){ // Handling code 501
      CurrentPage = String(FPSTR(ProgramPage));
    }else{
      if (Mode[1] == '1'){
        if (Mode[2] == '1'){ // Handle code 511
          CurrentPage = String(FPSTR(ProgramPage));
          CurrentPage +=  R"rawliteral(<p>Request to change slogan granted</p>)rawliteral";
        }
        else{ // Handle code 514
          CurrentPage = String(FPSTR(ProgramPage));
          CurrentPage +=  R"rawliteral(<p>Slogan updated</p>)rawliteral";
        }
      }
    }
  }
  else if (currentQueryCode == "#512\r\n"){ // handling reception after transmit
    Main_Slogan = Mode;
  }
  else if (Mode[0] == '7'){
    CurrentPage = String(FPSTR(EchoPage));
  }else if (prevQueryCode == "#700\r\n"){
    CurrentPage += R"rawliteral(<p>PIC :)rawliteral" + String(Mode) + R"rawliteral(</p>)rawliteral";
  }
  else
    CurrentPage +=  R"rawliteral(<p>Error or Timeout</p>)rawliteral";
}

void SendPicQuery(String query){
  if (commState == IDLE){
    Serial.print(query);
    commState = QUERY_SENT;
    responseStartTime = millis();
    picResponse = "";
    prevQueryCode = currentQueryCode;
    currentQueryCode = query;
  }
}

void handleCurrentPage(){
  // when request is made to ip adress, the current page number from PIC is displayed)
  String response = "<!DOCTYPE html><html>";
  response += MainHead;
  response += CurrentPage;
  response += "</html>";
  server.send(200, "text/html", response);
}

void handleHome(){//TODO: Change to start with a #
SendPicQuery("#000\r\n");
}

void handleSelectColor(){//TODO: Change to start with a #
  SendPicQuery("#100\r\n");
}

void handleCalibration(){//TODO: Change to start with a #
  SendPicQuery("#200\r\n");
}

void handleRace(){//TODO: Change to start with a #
  SendPicQuery("#300\r\n");
}

void handleSCRed(){//TODO: Change to start with a #
  SendPicQuery("#110\r\n");
}

void handleSCGreen(){//TODO: Change to start with a #
  SendPicQuery("#120\r\n");
}

void handleSCBlue(){//TODO: Change to start with a #
  SendPicQuery("#130\r\n");
}

void handleSCBlack(){//TODO: Change to start with a #
  SendPicQuery("#140\r\n");
}

void handleCalStatus(){//TODO: Change to start with a #
  SendPicQuery("#202\r\n");
}

void handleRaceStatus(){//TODO: Change to start with a #
  SendPicQuery("#302\r\n");
}

void handleRaceColor(){//TODO: Change to start with a #
  SendPicQuery("#303\r\n");
}

void handleDiagnostics(){
  SendPicQuery("#400\r\n");
}

void handleSensors(){
  SendPicQuery("#410\r\n");
}

void handleFoward(){
  SendPicQuery("#420\r\n");
}

void handleLeft(){
  SendPicQuery("#430\r\n");
}

void handleRight(){
  SendPicQuery("#440\r\n");
}

void handleStop(){
  SendPicQuery("#450\r\n");
}

void handleProgram(){
  SendPicQuery("#500\r\n");
}

void requestSloganChange(){
  SendPicQuery("#510\r\n");
}

void transmitSlogan(){
  if (server.hasArg("inputText")){
    Main_Slogan = server.arg("inputText");
  }
  String localMessage = Main_Slogan + "\r\n";
  SendPicQuery(localMessage);
}

void receiveSlogan(){
  SendPicQuery("#512\r\n");
}

void handleEcho(){
  SendPicQuery("#700\r\n");
}

void transmitEchoMessage(){
  String localMessage = "";
  if (server.hasArg("inputText")){
    localMessage = server.arg("inputText");
  }
  localMessage = localMessage + "\r\n";
  SendPicQuery(localMessage);
}

void checkPICResponse(){
  if (commState == QUERY_SENT || commState == WAITING_RESPONSE){
    if (Serial.available()) {// recieved a response from the PIC
      picResponse = Serial.readStringUntil('\n');//TODO: Change to take off the '#' at the beginning of the code
      picResponse.trim();
      if (picResponse.startsWith("#")) {
        picResponse.remove(0, 1); // Remove the first character
      }
      updateResponse(picResponse);
      commState = RESPONSE_RECEIVED;
    } else if (millis() - responseStartTime > responseTimeout && commState == WAITING_RESPONSE){ // time since query was sent, has been too long
      updateResponse("timeout");
      commState = RESPONSE_RECEIVED;
    } else if (commState == QUERY_SENT){
      commState = WAITING_RESPONSE;
    }
  }

  if (commState == RESPONSE_RECEIVED) {
    handleCurrentPage(); // 
    commState = IDLE; // Reset the state
  }
}