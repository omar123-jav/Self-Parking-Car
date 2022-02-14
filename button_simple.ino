#if 1

#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include <TouchScreen.h>
#define MINPRESSURE 200

#define MAXPRESSURE 1000
#include <FreeDefaultFonts.h>
#include <Wire.h>
#include <TEA5767.h>


TEA5767 radio = TEA5767();

// ALL Touch panels and wiring is DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
const int XP = 8, XM = A2, YP = A3, YM = 9; //ID=0x9341
const int TS_LEFT = 139, TS_RT = 903, TS_TOP = 109, TS_BOT = 901;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Adafruit_GFX_Button on_btn, off_btn, next_btn, prev_btn;

char *frequencies[3] = {"98.2", "93.7", "88.7"};
int idx = 0;
static double PRECISION = 0.01;

void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg);

char * dtoa(char *s, double n) {
    // handle special cases
    if (isnan(n)) {
        strcpy(s, "nan");
    } else if (isinf(n)) {
        strcpy(s, "inf");
    } else if (n == 0.0) {
        strcpy(s, "0");
    } else {
        int digit, m, m1;
        char *c = s;
        int neg = (n < 0);
        if (neg)
            n = -n;
        // calculate magnitude
        m = log10(n);
        int useExp = (m >= 14 || (neg && m >= 9) || m <= -9);
        if (neg)
            *(c++) = '-';
        // set up for scientific notation
        if (useExp) {
            if (m < 0)
               m -= 1.0;
            n = n / pow(10.0, m);
            m1 = m;
            m = 0;
        }
        if (m < 1.0) {
            m = 0;
        }
        // convert the number
        while (n > PRECISION || m >= 0) {
            double weight = pow(10.0, m);
            if (weight > 0 && !isinf(weight)) {
                digit = floor(n / weight);
                n -= (digit * weight);
                *(c++) = '0' + digit;
            }
            if (m == 0 && n > 0)
                *(c++) = '.';
            m--;
        }
        if (useExp) {
            // convert the exponent
            int i, j;
            *(c++) = 'e';
            if (m1 > 0) {
                *(c++) = '+';
            } else {
                *(c++) = '-';
                m1 = -m1;
            }
            m = 0;
            while (m1 > 0) {
                *(c++) = '0' + m1 % 10;
                m1 /= 10;
                m++;
            }
            c -= m;
            for (i = 0, j = m-1; i<j; i++, j--) {
                // swap without temporary
                c[i] ^= c[j];
                c[j] ^= c[i];
                c[i] ^= c[j];
            }
            c += m;
        }
        *(c) = '\0';
    }
    return s;
}
//#define ITOA(n) dtoa(char[20], (n) )

int pixel_x, pixel_y;     //Touch_getXY() updates global vars
bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
    }
    return pressed;
}

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xFF90
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

void setup(void)
{
//    Serial.begin(9600);
    uint16_t ID = tft.readID();
//    Serial.print("TFT ID = 0x");
//    Serial.println(ID, HEX);
//    Serial.println("Calibrate for your Touch Panel");
    if (ID == 0xD3D3) ID = 0x9486; // write-only shield
    tft.begin(ID);
    tft.setRotation(0);            //PORTRAIT
    tft.fillScreen(BLACK);
    on_btn.initButton(&tft,  60, 200, 100, 40, WHITE, CYAN, BLACK, "ON", 2);
    off_btn.initButton(&tft, 180, 200, 100, 40, WHITE, CYAN, BLACK, "OFF", 2);
    next_btn.initButton(&tft,  180, 250, 100, 40, WHITE, CYAN, BLACK, "Next", 2);
    prev_btn.initButton(&tft, 60, 250, 100, 40, WHITE, CYAN, BLACK, "Prev", 2);
    on_btn.drawButton(false);
    off_btn.drawButton(false);
    next_btn.drawButton(false);
    prev_btn.drawButton(false);
    tft.fillRect(40, 80, 160, 80, RED);
    showmsgXY(50, 100, 4, &FreeSevenSegNumFont, frequencies[idx]);
    Wire.begin();
    radio.setFrequency(atof(frequencies[idx]));
}

/* two buttons are quite simple
 */
void loop(void)
{
    bool down = Touch_getXY();
    on_btn.press(down && on_btn.contains(pixel_x, pixel_y));
    off_btn.press(down && off_btn.contains(pixel_x, pixel_y));
    next_btn.press(down && next_btn.contains(pixel_x, pixel_y));
    prev_btn.press(down && prev_btn.contains(pixel_x, pixel_y));
    if (on_btn.justReleased())
        on_btn.drawButton();
    if (off_btn.justReleased())
        off_btn.drawButton();
    if (next_btn.justReleased())
        next_btn.drawButton();
    if (prev_btn.justReleased())
        prev_btn.drawButton();
    if (on_btn.justPressed()) {
//      Wire.beginTransmission(5);
//        Wire.write(1);
//        Wire.endTransmission();
        on_btn.drawButton(true);
        radio.setMute(false);
//        tft.fillRect(40, 80, 160, 80, GREEN);
//        counter+=0.1;
    }
    if (off_btn.justPressed()) {
//      Wire.beginTransmission(5);
//        Wire.write(2);
//        Wire.endTransmission();
        off_btn.drawButton(true);
        radio.setMute(true);
        
//        tft.fillRect(40, 80, 160, 80, RED);
//        counter-=0.1;
    }
    if (next_btn.justPressed()) {
        next_btn.drawButton(true);
//        tft.fillRect(40, 80, 160, 80, GREEN);
//        counter+=0.1;
          idx++;
          if (idx > 2)
            idx = 0;
          radio.setFrequency(atof(frequencies[idx]));
          showmsgXY(50, 100, 4, &FreeSevenSegNumFont, frequencies[idx]);
    }
    if (prev_btn.justPressed()) {
        prev_btn.drawButton(true);
//        tft.fillRect(40, 80, 160, 80, RED);
//        counter-=0.1;
          idx--;
          if (idx < 0)
            idx = 2;
          radio.setFrequency(atof(frequencies[idx]));
          showmsgXY(50, 100, 4, &FreeSevenSegNumFont, frequencies[idx]);
    }
//    char s[5];
//    Serial.println(atof(counter));
    
}


void showmsgXY(int x, int y, int sz, const GFXfont *f, const char *msg)
{
    tft.fillRect(40, 80, 160, 80, RED);
    int16_t x1, y1;
    uint16_t wid, ht;
//    tft.setFont(f);
    tft.setCursor(x, y);
    tft.setTextColor(BLACK);
    tft.setTextSize(sz);
    tft.print(msg);
}

#endif
