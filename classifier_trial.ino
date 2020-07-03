#define CAMERA_MODEL_AI_THINKER

//#include <EloquentSVMSMO.h>
#include "model.h"
#include <esp_camera.h>
#include "camera_pins.h"



#define FRAME_SIZE FRAMESIZE_QQVGA
#define WIDTH 160
#define HEIGHT 120
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)

Eloquent::ML::Port::SVM pr;

//Eloquent::ML::Port::SVM predict;
//Eloquent::ML::Port::SVM
uint16_t prev_frame[H][W] = { 0 };
uint16_t current_frame[H][W] = { 0 };
uint16_t rgb_frame[H][W] = { 0 };
float features[H*W] = { 0 };


bool setup_camera(framesize_t);
bool capture_still();
void convert_to_rbg(uint8_t*, size_t);
void linearize_features();
void print_features();
void classify();
String classIdxToName(int);

/**
 *
 */
void setup() {
    Serial.begin(115200);
    Serial.println(setup_camera(FRAME_SIZE) ? "OK" : "ERR INIT");
}

/**
 *
 */
void loop() {
    if (!capture_still()) {
        Serial.println("Failed capture");
        delay(2000);

        return;
    }

    linearize_features();
    print_features();
    classify();
    delay(8000);
}


/**
 *
 */
bool setup_camera(framesize_t frameSize) {
    camera_config_t config;

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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = frameSize;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    bool ok = esp_camera_init(&config) == ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, frameSize);

    return ok;
}

/**
 * Capture image and do down-sampling
 */
bool capture_still() {
    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (!frame_buffer)
        return false;

    // set all 0s in current frame
    for (int y = 0; y < H; y++)
       { 
        for (int x = 0; x < W; x++){
          current_frame[y][x] = 0;
       }
       }    


    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        {
          for (int x = 0; x < W; x++){
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;
        }
      }
    
}


/**
 * Convert image to features vector
 */
void linearize_features() {
    size_t i = 0;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            features[i++] = current_frame[y][x];
        }
    }
}


/**
 *
 */
void print_features() {
    for (size_t i = 0; i < H*W; i++) {
        Serial.print(features[i]);
        Serial.print('\t');
    }
}

/**
 * Run the inference
 */
void classify() {
    Serial.println("Object: ");
    Serial.println(classIdxToName(pr.predict(features)));
}
String classIdxToName(int clas){
  switch (clas){
    case 0 : 
      return "mask";
      break;
    case 1 : 
      return "nomask";
      break;
}
}
