/***************************************************
 This is a library for the Multi-Touch Kit
 Designed and tested to work with Arduino Uno, MEGA2560, LilyPad(ATmega 328P)
 Note: Please remind to disconnect AREF pin from AVCC on Lilypad
 
 For details on using this library see the tutorial at:
 ----> https://hci.cs.uni-saarland.de/multi-touch-kit/
 
 Written by Narjes Pourjafarian, Jan Dickmann, Juergen Steimle (Saarland University), 
            Anusha Withana (University of Sydney), Joe Paradiso (MIT)
 MIT license, all text above must be included in any redistribution

 並列処理コードの実行用メインプログラム
 ****************************************************/

#include <MTKNanoESP32.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#define BLINK_PIN 8

//----- Multiplexer input pins (for UNO) -----
int s0 = 7;
int s1 = 6;
int s2 = 5;
int s3 = 4;
int s4 = 3;

int muxPins[5] = {s0, s1, s2, s3, s4};

int analogPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

int IMUPins[2] = {D11, D12};

rmt_data_t pwm_50duty[] = {
  {1, 1, 0, 0},  
  {0, 0, 0, 0}   
};

float kernel[3][3] = {
  {1.0, 2.0, 1.0},
  {2.0, 4.0, 2.0},
  {1.0, 2.0, 1.0}
};

//----- Number of receiver (RX) and transmitter (TX) lines -----
int RX_num = 8;
int TX_num = 23;

//----- Receive raw capacitance data or touch up/down states -----
boolean correct = true; // 補正を行う
int threshold = 250;  // Threshold for detecting touch down state (only required if raw_data = false). 
                    // Change this variable based on your sensor. (for more info. check the tutorial)
boolean toBLE = true; // BLE出力モード
int mode = 3; // 0:タッチ認識モード 1:最大タッチ位置のみ返す 2:指定した1点のみ返す 3:4段階タッチ認識モード

MTKNanoESP32 mtk;
BLEService bleService("8fc03459-5012-dcc8-f6c5-3425f9bb10ed");
BLECharacteristic bleCharacteristic("b70c5e87-fcd8-eec1-bf23-6e98d6b38730", BLERead | BLENotify, 64);
int timelineDatas[23][8][10];
int noise[23][8];
int writeID;
int writeIDold;

// 時間測定用
unsigned long startTime;
unsigned long count;

// 宣言だけやっておく
void taskRead(void* arg);
void taskBLE(void* arg);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Lチカ用
  digitalWrite(LED_BUILTIN, HIGH);
  // シリアル通信
  Serial.begin(115200);
  sleep(10); // 接続待ち エラー落ち時に再コンパイルできるように長めに取った
  Serial.println("S");

  // アナログ入力の範囲設定
  analogSetAttenuation(ADC_2_5db); 

  // RMTの処理 10MHz50%
  rmtInit(BLINK_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000);
  rmtWriteLooping(BLINK_PIN, pwm_50duty, RMT_SYMBOLS_OF(pwm_50duty));
  Serial.println("rmt");

  if (toBLE) {
    if(BLE.begin()){
        Serial.println("BLEStart");
    }
    BLE.setDeviceName("ArduinoNanoESP32");
    bleService.addCharacteristic(bleCharacteristic);
    BLE.addService(bleService);
    BLE.setLocalName("CylindricalController");
    BLE.setAdvertisedService(bleService);
    BLE.advertise();
    // タイムラインの値に0を設定
    // ノイズの値に1を設定
    for (size_t tx = 0; tx < TX_num; tx++){
        for (size_t rx = 0; rx < RX_num; rx++){
            for (size_t i = 0; i < 10; i++){
                timelineDatas[tx][rx][i] = 0;
            }
            noise[tx][rx] = 1;
        }
    }
  }

  for (size_t i = 0; i < 3; i++){
    for (size_t j = 0; j < 3; j++){
      kernel[i][j] = kernel[i][j] / 16;
    }
  }

  // センサのセットアップ
  mtk.setup_sensor(muxPins,correct,threshold, 5, analogPins, toBLE, IMUPins, timelineDatas, &writeID, noise);
  Serial.println("setup");

  startTime = millis(); // 計測開始時刻の初期化
  count = 0; // 処理回数カウンタの初期化
  Serial.println("count");

  // // タスクの作成
  xTaskCreatePinnedToCore(
    taskRead, // 変数名
    "taskRead",  // 人間が読む名前
    4096,  // スタックサイズ，足りない場合→`uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    NULL,  // 引数
    0,  // 優先度
    NULL,  // nullでいいらしい
    0  // コアの番号
  );

  if(toBLE){
    // BLEタスクの作成
    xTaskCreatePinnedToCore(
      taskBLE, // 変数名
      "taskBLE",  // 人間が読む名前
      4096,  // スタックサイズ，足りない場合→`uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
      NULL,  // 引数
      1,  // 優先度
      NULL,  // nullでいいらしい
      1  // コアの番号
    );
  }
  
  Serial.println("task");
  digitalWrite(LED_BUILTIN, LOW); 
}

// loopは空
void loop() { }

// 電圧取得処理&シリアル出力
void taskRead(void* arg){
  for (;;) {
    mtk.read();
    //Serial.println(writeID);
    vTaskDelay(2); // ディレイをかけて停止を防止
    // count++; // 処理が 1 回実行されるたびにカウント
    // if (millis() - startTime >= 1000) { // 1 秒（1000 ミリ秒）経過したら処理回数を出力する
    //   Serial.print("概算fps: ");
    //   Serial.println(count);
    //   count = 0; // カウンタをリセット
    //   startTime = millis(); // 計測開始時刻をリセット
    // }
  }
  vTaskDelay(1000);
}


// BLE用,データ処理も行う
void taskBLE(void* arg){
  for(;;) {
    BLEDevice central = BLE.central(); // 接続の有無を判定
    if(central){
      float RCs[23][8]; // 各格子点のRCフィルタ適用後の値
      float convolutions[23][8]; // 畳み込み後の値
      float alpha = 0.8; // rcフィルタのalpha
      for (size_t tx = 0; tx < TX_num; tx++){
        for (size_t rx = 0; rx < RX_num; rx++){
          RCs[tx][rx] = timelineDatas[tx][rx][writeID];
        }
      }
      uint8_t dataStorage[23]; // mode0用 // 送信用データストレージ 23*8bit
      int maxtx, maxrx; int maxVal = 0; // mode1用 // 最大値のID // 全範囲の最高値
      int TX = 11, RX = 3; float rcA = 0; float rcB = 0; // mode2用 // 送信する場所 // rcフィルタ // 実験用rcフィルタ
      uint16_t dataStorage16[23]; uint8_t val2bit; // mode3用 // 送信用データストレージ 23*16bit // 2bit値の一時保存
      writeIDold =  writeID;
      while (central.connected()) { // 接続中の処理 
        vTaskDelay(5); // 高頻度
        if(writeIDold == writeID){ continue; } // 読み取りが行われていないなら読み取らない
        writeIDold = writeID; // 更新
        for (size_t tx = 0; tx < TX_num; tx++){
          for (size_t rx = 0; rx < RX_num; rx++){
            int val = timelineDatas[tx][rx][writeID]; // 最新値
            int noiseVal = noise[tx][rx]; // ノイズ
            if(val > 100){ // フィルタ処理 RCフィルタ(n未満の値は排除)
              RCs[tx][rx] = alpha * RCs[tx][rx] + (1-alpha) * (float)val; // フィルタ後の値
            }
            if(mode == 0){
              if (RCs[tx][rx] > noiseVal*1.1){ // 値が閾値より高い
                dataStorage[tx] |=  (1 << (rx));
              }else{
                dataStorage[tx] &= ~(1 << rx);
              }
            }else if(mode == 1){
              if(RCs[tx][rx] > maxVal){
                maxtx = tx;
                maxrx = rx;
                maxVal = RCs[tx][rx];
              }
            }
          }
        }

        if(mode == 3 || mode==2){
          // 畳み込み処理
          for (size_t tx = 0; tx < TX_num; tx++){
            for (size_t rx = 0; rx < RX_num; rx++){
              // // 畳み込み処理の部分 一旦凍結
              // float sumVals = 0.0; // 合計値を記録する
              // int sumNum = 0; // 最後に割る用の回数カウント
              // // 上下左右についてのループ処理
              // for(size_t i = 0; i < 3; i++){
              //   //Serial.print(i);
              //   if(tx+i-1 < 0 || tx+i-1 >= TX_num){ continue; } // 配列外処理
              //   for(size_t j = 0; j < 3; j++){
              //     if(rx+j-1 < 0 || rx+j-1 >= RX_num){ continue; } // 配列外処理
              //     sumVals += RCs[tx+i-1][rx+i-1] * kernel[i][j];
              //     Serial.print(RCs[tx+i-1][rx+i-1]);
              //     Serial.print(",");
              //     Serial.print(kernel[i][j]);
              //     Serial.print(",");
              //     Serial.println(sumVals);
              //     sumNum += 1;
              //   }
              // }
              // //Serial.println(sumVals);
              
              // float convVal = sumVals / sumNum; // 平均値を入れる
              float convVal = RCs[tx][rx];
              int noiseVal = noise[tx][rx];
              // 判定 0~3への割り当て
              // 4段階で下から分ける
              if(convVal < noiseVal* 0.9){
                val2bit = 0;
              }else if(convVal < noiseVal*1.2){
                val2bit = 1;
              }else if(convVal < noiseVal*1.7){
                val2bit = 2;
              }else{
                val2bit = 3;
              }
              dataStorage16[tx] &= ~(0x03 << rx*2); // 既存ビットをクリア (0x03=2ビット分のマスク)
              dataStorage16[tx] |= ((uint16_t)(val2bit & 0x03) << rx*2); // 新しい値を設定

              if(mode == 2){
                if(tx == TX && rx == RX){
                  uint16_t outputRC[3] = {(uint16_t)timelineDatas[tx][rx][writeID], (uint16_t)RCs[tx][rx], (uint16_t)convVal};
                  bleCharacteristic.writeValue(outputRC, 6);
                }
              }
            }
          }
        }

        // 送信処理など
        if(mode == 0){
          bleCharacteristic.writeValue(dataStorage, 23);
        }else if(mode == 1){ // 閾値設定もここで
          if(maxVal > 700){
            uint8_t maxID[3] = {maxtx, maxrx, maxVal};
            bleCharacteristic.writeValue(maxID, 3);
          }else{
            uint8_t none = 0;
            bleCharacteristic.writeValue(none, 1);
          }
        }else if(mode == 3){
          bleCharacteristic.writeValue(dataStorage16, 46);
        }
      }
    }
    vTaskDelay(1000);
  }
}