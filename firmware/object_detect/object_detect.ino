// In-browser object detection for the AI-Thinker ESP32-CAM (OV2640).
//
// The ESP32 only serves JPEG stills; detection runs in the browser with
// TensorFlow.js COCO-SSD, so the page needs internet access to load the model.
// Open http://<ip>/ and press Start Detect. Counts of the chosen object are
// sent back and printed over serial as "<object> = <count>".
//
// Ported to Arduino-ESP32 3.x: the LEDC API changed (ledcSetup/ledcAttachPin
// are gone), and LINE Notify was removed because the service shut down in
// March 2025.

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD. Gitignored; see secrets.h.example.

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Fallback access point, always started alongside the station connection.
// The SSID is prefixed with the board's IP so you can tell which address to open.
#ifndef AP_SSID
#define AP_SSID     "ESP32-CAM"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "esp32cam"   // At least 8 characters, or softAP refuses it.
#endif
const char* apssid     = AP_SSID;
const char* appassword = AP_PASSWORD;

String Feedback="";

String Command="",cmd="",P1="",P2="",P3="",P4="",P5="",P6="",P7="",P8="",P9="";

byte ReceiveState=0,cmdState=1,strState=1,questionstate=0,equalstate=0,semicolonstate=0;

// AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// The camera drives XCLK from LEDC channel 0 / timer 0 through the IDF, which
// the Arduino allocator cannot see. Pin explicit channels well clear of it.
#define FLASH_PIN      4
#define FLASH_CHANNEL  4
#define PWM_CHANNEL    5

WiFiServer server(80);

void flashWrite(int val) {
  ledcDetach(FLASH_PIN);
  ledcAttachChannel(FLASH_PIN, 5000, 8, FLASH_CHANNEL);
  ledcWrite(FLASH_PIN, val);
}

void ExecuteCommand()
{
  if (cmd!="getstill") {
    Serial.println("cmd= "+cmd+" ,P1= "+P1+" ,P2= "+P2+" ,P3= "+P3+" ,P4= "+P4+" ,P5= "+P5+" ,P6= "+P6+" ,P7= "+P7+" ,P8= "+P8+" ,P9= "+P9);
    Serial.println("");
  }

  if (cmd=="your cmd") {
    // You can do anything.
    // Feedback="<font color=\"red\">Hello World</font>";
  }
  else if (cmd=="ip") {
    Feedback="AP IP: "+WiFi.softAPIP().toString();
    Feedback+=", ";
    Feedback+="STA IP: "+WiFi.localIP().toString();
  }
  else if (cmd=="mac") {
    Feedback="STA MAC: "+WiFi.macAddress();
  }
  else if (cmd=="resetwifi") {
    WiFi.begin(P1.c_str(), P2.c_str());
    Serial.print("Connecting to ");
    Serial.println(P1);
    long int StartTime=millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        if ((StartTime+5000) < millis()) break;
    }
    Serial.println("");
    Serial.println("STAIP: "+WiFi.localIP().toString());
    Feedback="STAIP: "+WiFi.localIP().toString();
  }
  else if (cmd=="restart") {
    ESP.restart();
  }
  else if (cmd=="digitalwrite") {
    ledcDetach(P1.toInt());
    pinMode(P1.toInt(), OUTPUT);
    digitalWrite(P1.toInt(), P2.toInt());
  }
  else if (cmd=="analogwrite") {
    if (P1.toInt()==FLASH_PIN) {
      flashWrite(P2.toInt());
    }
    else {
      ledcDetach(P1.toInt());
      ledcAttachChannel(P1.toInt(), 5000, 8, PWM_CHANNEL);
      ledcWrite(P1.toInt(), P2.toInt());
    }
  }
  else if (cmd=="flash") {
    flashWrite(P1.toInt());
  }
  else if (cmd=="framesize") {
    sensor_t * s = esp_camera_sensor_get();
    if (P1=="QQVGA")
      s->set_framesize(s, FRAMESIZE_QQVGA);
    else if (P1=="HQVGA")
      s->set_framesize(s, FRAMESIZE_HQVGA);
    else if (P1=="QVGA")
      s->set_framesize(s, FRAMESIZE_QVGA);
    else if (P1=="CIF")
      s->set_framesize(s, FRAMESIZE_CIF);
    else if (P1=="VGA")
      s->set_framesize(s, FRAMESIZE_VGA);
    else if (P1=="SVGA")
      s->set_framesize(s, FRAMESIZE_SVGA);
    else if (P1=="XGA")
      s->set_framesize(s, FRAMESIZE_XGA);
    else if (P1=="SXGA")
      s->set_framesize(s, FRAMESIZE_SXGA);
    else if (P1=="UXGA")
      s->set_framesize(s, FRAMESIZE_UXGA);
    else
      s->set_framesize(s, FRAMESIZE_QVGA);
  }
  else if (cmd=="quality") {
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();
    s->set_quality(s, val);
  }
  else if (cmd=="contrast") {
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();
    s->set_contrast(s, val);
  }
  else if (cmd=="brightness") {
    sensor_t * s = esp_camera_sensor_get();
    int val = P1.toInt();
    s->set_brightness(s, val);
  }
  else if (cmd=="serial") {
    Serial.println(P1);
  }
  else if (cmd=="detectCount") {
    P1.replace("%20"," ");   // "traffic light" arrives URL-encoded.
    Serial.println(P1+" = "+P2);
  }
  else if (cmd=="tcp") {
    String domain=P1;
    int port=P2.toInt();
    String request=P3;
    int wait=P4.toInt();      // wait = 0 or 1

    if ((port==443)||(domain.indexOf("https")==0)||(domain.indexOf("HTTPS")==0))
      Feedback=tcp_https(domain,request,port,wait);
    else
      Feedback=tcp_http(domain,request,port,wait);
  }
  else {
    Feedback="Command is not defined.";
  }
  if (Feedback=="") Feedback=Command;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config = {};   // Zeroed: newer fields such as jpeg_buffer_size must not be stack garbage.
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;   // Detect on the newest frame, not a stale buffered one.
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);  //UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  ledcAttachChannel(FLASH_PIN, 5000, 8, FLASH_CHANNEL);

  WiFi.mode(WIFI_AP_STA);

  //WiFi.config(IPAddress(192, 168, 201, 100), IPAddress(192, 168, 201, 2), IPAddress(255, 255, 255, 0));

  WiFi.begin(ssid, password);

  delay(1000);
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  long int StartTime=millis();
  while (WiFi.status() != WL_CONNECTED)
  {
      delay(500);
      if ((StartTime+10000) < millis()) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.softAP((WiFi.localIP().toString()+"_"+(String)apssid).c_str(), appassword);
    Serial.println("");
    Serial.println("STAIP address: ");
    Serial.println(WiFi.localIP());

    for (int i=0;i<5;i++) {
      ledcWrite(FLASH_PIN,10);
      delay(200);
      ledcWrite(FLASH_PIN,0);
      delay(200);
    }
  }
  else {
    WiFi.softAP((WiFi.softAPIP().toString()+"_"+(String)apssid).c_str(), appassword);

    for (int i=0;i<2;i++) {
      ledcWrite(FLASH_PIN,10);
      delay(1000);
      ledcWrite(FLASH_PIN,0);
      delay(1000);
    }
  }

  //WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  Serial.println("");
  Serial.println("APIP address: ");
  Serial.println(WiFi.softAPIP());

  ledcWrite(FLASH_PIN, 0);

  server.begin();
}

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
  <!DOCTYPE html>
  <head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <script src="https:\/\/ajax.googleapis.com/ajax/libs/jquery/1.8.0/jquery.min.js"></script>
  <script src="https:\/\/cdn.jsdelivr.net/npm/@tensorflow/tfjs@1.3.1/dist/tf.min.js"> </script>
  <script src="https:\/\/cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.1.0"> </script>
  </head><body>
  <img id="ShowImage" src="" style="display:none">
  <canvas id="canvas" width="0" height="0"></canvas>
  <table>
  <tr>
    <td><input type="button" id="restart" value="Restart"></td>
    <td colspan="2"><input type="button" id="getStill" value="Start Detect" style="display:none"></td>
  </tr>
  <tr>
    <td>Object</td>
    <td colspan="2">
        <select id="object" onchange="count.innerHTML='';">
          <option value="person">person</option>
          <option value="bicycle">bicycle</option>
          <option value="car">car</option>
          <option value="motorcycle">motorcycle</option>
          <option value="airplane">airplane</option>
          <option value="bus">bus</option>
          <option value="train">train</option>
          <option value="truck">truck</option>
          <option value="boat">boat</option>
          <option value="traffic light">traffic light</option>
          <option value="fire hydrant">fire hydrant</option>
          <option value="stop sign">stop sign</option>
          <option value="parking meter">parking meter</option>
          <option value="bench">bench</option>
          <option value="bird">bird</option>
          <option value="cat">cat</option>
          <option value="dog">dog</option>
          <option value="horse">horse</option>
          <option value="sheep">sheep</option>
          <option value="cow">cow</option>
          <option value="elephant">elephant</option>
          <option value="bear">bear</option>
          <option value="zebra">zebra</option>
          <option value="giraffe">giraffe</option>
          <option value="backpack">backpack</option>
          <option value="umbrella">umbrella</option>
          <option value="handbag">handbag</option>
          <option value="tie">tie</option>
          <option value="suitcase">suitcase</option>
          <option value="frisbee">frisbee</option>
          <option value="skis">skis</option>
          <option value="snowboard">snowboard</option>
          <option value="sports ball">sports ball</option>
          <option value="kite">kite</option>
          <option value="baseball bat">baseball bat</option>
          <option value="baseball glove">baseball glove</option>
          <option value="skateboard">skateboard</option>
          <option value="surfboard">surfboard</option>
          <option value="tennis racket">tennis racket</option>
          <option value="bottle">bottle</option>
          <option value="wine glass">wine glass</option>
          <option value="cup">cup</option>
          <option value="fork">fork</option>
          <option value="knife">knife</option>
          <option value="spoon">spoon</option>
          <option value="bowl">bowl</option>
          <option value="banana">banana</option>
          <option value="apple">apple</option>
          <option value="sandwich">sandwich</option>
          <option value="orange">orange</option>
          <option value="broccoli">broccoli</option>
          <option value="carrot">carrot</option>
          <option value="hot dog">hot dog</option>
          <option value="pizza">pizza</option>
          <option value="donut">donut</option>
          <option value="cake">cake</option>
          <option value="chair">chair</option>
          <option value="couch">couch</option>
          <option value="potted plant">potted plant</option>
          <option value="bed">bed</option>
          <option value="dining table">dining table</option>
          <option value="toilet">toilet</option>
          <option value="tv">tv</option>
          <option value="laptop">laptop</option>
          <option value="mouse">mouse</option>
          <option value="remote">remote</option>
          <option value="keyboard">keyboard</option>
          <option value="cell phone">cell phone</option>
          <option value="microwave">microwave</option>
          <option value="oven">oven</option>
          <option value="toaster">toaster</option>
          <option value="sink">sink</option>
          <option value="refrigerator">refrigerator</option>
          <option value="book">book</option>
          <option value="clock">clock</option>
          <option value="vase">vase</option>
          <option value="scissors">scissors</option>
          <option value="teddy bear">teddy bear</option>
          <option value="hair drier">hair drier</option>
          <option value="toothbrush">toothbrush</option>
        </select>
    </td>
    <td><span id="count" style="color:red"><span></td>
  </tr>
  <tr>
    <td>ScoreLimit</td>
    <td colspan="2">
      <select id="score">
        <option value="1.0">1</option>
        <option value="0.9">0.9</option>
        <option value="0.8">0.8</option>
        <option value="0.7">0.7</option>
        <option value="0.6">0.6</option>
        <option value="0.5">0.5</option>
        <option value="0.4">0.4</option>
        <option value="0.3">0.3</option>
        <option value="0.2">0.2</option>
        <option value="0.1">0.1</option>
        <option value="0" selected="selected">0</option>
      </select>
    </td>
  </tr>
  <tr>
    <td>MirrorImage</td>
    <td colspan="2">
      <select id="mirrorimage">
        <option value="1">yes</option>
        <option value="0">no</option>
      </select>
    </td>
  </tr>
  <tr>
    <td>Resolution</td>
    <td colspan="2">
      <select id="framesize">
        <option value="UXGA">UXGA(1600x1200)</option>
        <option value="SXGA">SXGA(1280x1024)</option>
        <option value="XGA">XGA(1024x768)</option>
        <option value="SVGA">SVGA(800x600)</option>
        <option value="VGA">VGA(640x480)</option>
        <option value="CIF">CIF(400x296)</option>
        <option value="QVGA" selected="selected">QVGA(320x240)</option>
        <option value="HQVGA">HQVGA(240x176)</option>
        <option value="QQVGA">QQVGA(160x120)</option>
      </select>
    </td>
  </tr>
  <tr>
    <td>Flash</td>
    <td colspan="2"><input type="range" id="flash" min="0" max="255" value="0"></td>
  </tr>
  <tr>
    <td>Quality</td>
    <td colspan="2"><input type="range" id="quality" min="10" max="63" value="10"></td>
  </tr>
  <tr>
    <td>Brightness</td>
    <td colspan="2"><input type="range" id="brightness" min="-2" max="2" value="0"></td>
  </tr>
  <tr>
    <td>Contrast</td>
    <td colspan="2"><input type="range" id="contrast" min="-2" max="2" value="0"></td>
  </tr>

  </table>
  <div id="result" style="color:red"><div>
  </body>
  </html>

  <script>
    var getStill = document.getElementById('getStill');
    var ShowImage = document.getElementById('ShowImage');
    var canvas = document.getElementById("canvas");
    var context = canvas.getContext("2d");
    var object = document.getElementById('object');
    var score = document.getElementById("score");
    var mirrorimage = document.getElementById("mirrorimage");
    var count = document.getElementById('count');
    var result = document.getElementById('result');
    var flash = document.getElementById('flash');
    var lastValue="";
    var myTimer;
    var restartCount=0;
    var Model;
    getStill.onclick = function (event) {
      clearInterval(myTimer);
      myTimer = setInterval(function(){error_handle();},5000);
      ShowImage.src=location.origin+'/?getstill='+Math.random();
    }
    function error_handle() {
      restartCount++;
      clearInterval(myTimer);
      if (restartCount<=2) {
        result.innerHTML = "Get still error. <br>Restart ESP32-CAM "+restartCount+" times.";
        myTimer = setInterval(function(){getStill.click();},10000);
      }
      else
        result.innerHTML = "Get still error. <br>Please close the page and check ESP32-CAM.";
    }
    ShowImage.onload = function (event) {
      clearInterval(myTimer);
      restartCount=0;
      canvas.setAttribute("width", ShowImage.width);
      canvas.setAttribute("height", ShowImage.height);
      if (mirrorimage.value==1) {
        context.translate((canvas.width + ShowImage.width) / 2, 0);
        context.scale(-1, 1);
        context.drawImage(ShowImage, 0, 0, ShowImage.width, ShowImage.height);
        context.setTransform(1, 0, 0, 1, 0, 0);
      }
      else
        context.drawImage(ShowImage,0,0,ShowImage.width,ShowImage.height);
      if (Model) {
        DetectImage();
      }
    }

    restart.onclick = function (event) {
      fetch(location.origin+'/?restart=stop');
    }
    framesize.onchange = function (event) {
      fetch(document.location.origin+'/?framesize='+this.value+';stop');
    }
    flash.onchange = function (event) {
      fetch(location.origin+'/?flash='+this.value+';stop');
    }
    quality.onchange = function (event) {
      fetch(document.location.origin+'/?quality='+this.value+';stop');
    }
    brightness.onchange = function (event) {
      fetch(document.location.origin+'/?brightness='+this.value+';stop');
    }
    contrast.onchange = function (event) {
      fetch(document.location.origin+'/?contrast='+this.value+';stop');
    }

    function ObjectDetect() {
      result.innerHTML = "Please wait for loading model.";
      cocoSsd.load().then(cocoSsd_Model => {
        Model = cocoSsd_Model;
        result.innerHTML = "";
        getStill.style.display = "block";
      });
    }

    function DetectImage() {
      Model.detect(canvas).then(Predictions => {
        var s = (canvas.width>canvas.height)?canvas.width:canvas.height;
        var objectCount=0;

        if (Predictions.length>0) {
          result.innerHTML = "";
          for (var i=0;i<Predictions.length;i++) {
            const x = Predictions[i].bbox[0];
            const y = Predictions[i].bbox[1];
            const width = Predictions[i].bbox[2];
            const height = Predictions[i].bbox[3];
            context.lineWidth = Math.round(s/200);
            context.strokeStyle = "#00FFFF";
            context.beginPath();
            context.rect(x, y, width, height);
            context.stroke();
            context.lineWidth = "2";
            context.fillStyle = "red";
            context.font = Math.round(s/30) + "px Arial";
            context.fillText(Predictions[i].class, x, y);
            result.innerHTML+= "[ "+i+" ] "+Predictions[i].class+", "+Math.round(Predictions[i].score*100)+"%, "+Math.round(x)+", "+Math.round(y)+", "+Math.round(width)+", "+Math.round(height)+"<br>";

            if (Predictions[i].class==object.value&&Predictions[i].score>=score.value) {
              objectCount++;
            }
          }
          count.innerHTML = objectCount;
        }
        else {
          result.innerHTML = "Unrecognizable";
          count.innerHTML = "0";
        }

        lastValue = count.innerHTML;
        if (objectCount>0) {
          $.ajax({url: document.location.origin+'/?detectCount='+object.value+';'+String(objectCount)+';stop', async: false});
        }
        try {
          document.createEvent("TouchEvent");
          setTimeout(function(){getStill.click();},250);
        }
        catch(e) {
          setTimeout(function(){getStill.click();},150);
        }
      });
    }
    function getFeedback(target) {
      var data = $.ajax({
      type: "get",
      dataType: "text",
      url: target,
      success: function(response)
        {
          result.innerHTML = response;
        },
        error: function(exception)
        {
          result.innerHTML = 'fail';
        }
      });
    }
    window.onload = function () { ObjectDetect(); }
  </script>
)rawliteral";

void loop() {
  Feedback="";Command="";cmd="";P1="";P2="";P3="";P4="";P5="";P6="";P7="";P8="";P9="";
  ReceiveState=0,cmdState=1,strState=1,questionstate=0,equalstate=0,semicolonstate=0;

  WiFiClient client = server.available();

  if (client) {
    String currentLine = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        getCommand(c);

        if (c == '\n') {
          if (currentLine.length() == 0) {

            if (cmd=="getstill") {

              camera_fb_t * fb = NULL;
              fb = esp_camera_fb_get();
              if(!fb) {
                Serial.println("Camera capture failed");
                delay(1000);
                ESP.restart();
              }

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: image/jpeg");
              client.println("Content-Disposition: form-data; name=\"imageFile\"; filename=\"picture.jpg\"");
              client.println("Content-Length: " + String(fb->len));
              client.println("Connection: close");
              client.println();

              uint8_t *fbBuf = fb->buf;
              size_t fbLen = fb->len;
              for (size_t n=0;n<fbLen;n=n+1024) {
                if (n+1024<fbLen) {
                  client.write(fbBuf, 1024);
                  fbBuf += 1024;
                }
                else if (fbLen%1024>0) {
                  size_t remainder = fbLen%1024;
                  client.write(fbBuf, remainder);
                }
              }

              esp_camera_fb_return(fb);
            }
            else {

              client.println("HTTP/1.1 200 OK");
              client.println("Access-Control-Allow-Headers: Origin, X-Requested-With, Content-Type, Accept");
              client.println("Access-Control-Allow-Methods: GET,POST,PUT,DELETE,OPTIONS");
              client.println("Content-Type: text/html; charset=utf-8");
              client.println("Access-Control-Allow-Origin: *");
              client.println("Connection: close");
              client.println();

              String Data="";
              if (cmd!="")
                Data = Feedback;
              else {
                Data = String((const char *)INDEX_HTML);
              }
              int Index;
              for (Index = 0; Index < Data.length(); Index = Index+1000) {
                client.print(Data.substring(Index, Index+1000));
              }

              client.println();
            }

            Feedback="";
            break;
          } else {
            currentLine = "";
          }
        }
        else if (c != '\r') {
          currentLine += c;
        }

        if ((currentLine.indexOf("/?")!=-1)&&(currentLine.indexOf(" HTTP")!=-1)) {
          if (Command.indexOf("stop")!=-1) {   // http://192.168.xxx.xxx/?cmd=aaa;bbb;ccc;stop
            client.println();
            client.println();
            client.stop();
          }
          currentLine="";
          Feedback="";
          ExecuteCommand();
        }
      }
    }
    delay(1);
    client.stop();
  }
}

void getCommand(char c)
{
  if (c=='?') ReceiveState=1;
  if ((c==' ')||(c=='\r')||(c=='\n')) ReceiveState=0;

  if (ReceiveState==1)
  {
    Command=Command+String(c);

    if (c=='=') cmdState=0;
    if (c==';') strState++;

    if ((cmdState==1)&&((c!='?')||(questionstate==1))) cmd=cmd+String(c);
    if ((cmdState==0)&&(strState==1)&&((c!='=')||(equalstate==1))) P1=P1+String(c);
    if ((cmdState==0)&&(strState==2)&&(c!=';')) P2=P2+String(c);
    if ((cmdState==0)&&(strState==3)&&(c!=';')) P3=P3+String(c);
    if ((cmdState==0)&&(strState==4)&&(c!=';')) P4=P4+String(c);
    if ((cmdState==0)&&(strState==5)&&(c!=';')) P5=P5+String(c);
    if ((cmdState==0)&&(strState==6)&&(c!=';')) P6=P6+String(c);
    if ((cmdState==0)&&(strState==7)&&(c!=';')) P7=P7+String(c);
    if ((cmdState==0)&&(strState==8)&&(c!=';')) P8=P8+String(c);
    if ((cmdState==0)&&(strState>=9)&&((c!=';')||(semicolonstate==1))) P9=P9+String(c);

    if (c=='?') questionstate=1;
    if (c=='=') equalstate=1;
    if ((strState>=9)&&(c==';')) semicolonstate=1;
  }
}

String tcp_http(String domain,String request,int port,byte wait)
{
    WiFiClient client_tcp;

    if (client_tcp.connect(domain.c_str(), port))
    {
      Serial.println("GET " + request);
      client_tcp.println("GET " + request + " HTTP/1.1");
      client_tcp.println("Host: " + domain);
      client_tcp.println("Connection: close");
      client_tcp.println();

      String getResponse="",Feedback="";
      boolean state = false;
      int waitTime = 3000;   // timeout 3 seconds
      long startTime = millis();
      while ((startTime + waitTime) > millis())
      {
        while (client_tcp.available())
        {
            char c = client_tcp.read();
            if (state==true) Feedback += String(c);
            if (c == '\n')
            {
              if (getResponse.length()==0) state=true;
              getResponse = "";
            }
            else if (c != '\r')
              getResponse += String(c);
            if (wait==1)
              startTime = millis();
         }
         if (wait==0)
          if ((state==true)&&(Feedback.length()!= 0)) break;
      }
      client_tcp.stop();
      return Feedback;
    }
    else
      return "Connection failed";
}

String tcp_https(String domain,String request,int port,byte wait)
{
    WiFiClientSecure client_tcp;
    client_tcp.setInsecure();

    if (client_tcp.connect(domain.c_str(), port))
    {
      Serial.println("GET " + request);
      client_tcp.println("GET " + request + " HTTP/1.1");
      client_tcp.println("Host: " + domain);
      client_tcp.println("Connection: close");
      client_tcp.println();

      String getResponse="",Feedback="";
      boolean state = false;
      int waitTime = 3000;   // timeout 3 seconds
      long startTime = millis();
      while ((startTime + waitTime) > millis())
      {
        while (client_tcp.available())
        {
            char c = client_tcp.read();
            if (state==true) Feedback += String(c);
            if (c == '\n')
            {
              if (getResponse.length()==0) state=true;
              getResponse = "";
            }
            else if (c != '\r')
              getResponse += String(c);
            if (wait==1)
              startTime = millis();
         }
         if (wait==0)
          if ((state==true)&&(Feedback.length()!= 0)) break;
      }
      client_tcp.stop();
      return Feedback;
    }
    else
      return "Connection failed";
}
