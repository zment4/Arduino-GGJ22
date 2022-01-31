// Arduino Pro Micro

#include "fastrand.h"
#include "chargen.h"

// RS, WR, RD, RST
#define PIN_ILI_RS   7
#define PIN_ILI_WR   5
#define PIN_ILI_RD   6
#define PIN_ILI_RST  4

// Would it be easier programming wise if these were swapped?
// F4-F7 -> DB10-DB13  
// D0-D3 -> DB14-DB17
#define PIN_ILI_DB10 21
#define PIN_ILI_DB11 20
#define PIN_ILI_DB12 19
#define PIN_ILI_DB13 18
#define PIN_ILI_DB14 3
#define PIN_ILI_DB15 2
#define PIN_ILI_DB16 0
#define PIN_ILI_DB17 1

// Buttons
#define PIN_BUTTON1  9
#define PIN_BUTTON2  10

// ILI Commands
#define ILI_CMD_NOP                       0x00
#define ILI_CMD_SLEEPOUT                  0x11
#define ILI_CMD_DISPLAYON                 0x29
#define ILI_CMD_WRITEDISPLAYBRIGHTNESS    0x51
#define ILI_CMD_WRITECTRLDISPLAY          0x53
#define ILI_CMD_MEMORYWRITE               0x2C
#define ILI_CMD_DISPLAYINVERSIONON        0x21
#define ILI_CMD_DISPLAYINVERSIONOFF       0x20
#define ILI_CMD_CASET                     0x2A
#define ILI_CMD_PASET                     0x2B
#define ILI_CMD_MADCTL                    0x36

// ILI Masks
#define ILI_MASK_BCTRL 0b00100000
#define ILI_MASK_DD    0b00001000
#define ILI_MASK_BL    0b00000100
#define ILI_MASK_MY    0b10000000
#define ILI_MASK_MX    0b01000000
#define ILI_MASK_MV    0b00100000
#define ILI_MASK_ML    0b00010000
#define ILI_MASK_BGR   0b00001000
#define ILI_MASK_MH    0b00000100

void resetILI();
void toggleILIWr();
void setILIWr(uint8_t state);
void setILIRd(uint8_t state);
void setILIRs(uint8_t state);
void setILICtrlHigh();
void writeILIByte(uint8_t byteWrite);
uint8_t readILIByte();
void sendILICommand(uint8_t regAddress);
void setDataBusOutput();
void setDataBusInput();
void setDataBus(uint8_t data);
uint8_t readDataBus();
void writeCharToScreen(uint8_t cx, uint8_t cy, char character);

uint8_t databusPins[8] = {
  PIN_ILI_DB10,
  PIN_ILI_DB11,
  PIN_ILI_DB12,
  PIN_ILI_DB13,
  PIN_ILI_DB14,
  PIN_ILI_DB15,
  PIN_ILI_DB16,
  PIN_ILI_DB17
};

#define SCR_CELL_WIDTH  40
#define SCR_CELL_HEIGHT 30

#define CELL_WIDTH      8
#define CELL_HEIGHT     8

#define RANDOM_FLIP_MICROS 400

volatile int whites = (SCR_CELL_WIDTH / 2) * SCR_CELL_HEIGHT;
volatile int blacks = (SCR_CELL_WIDTH / 2) * SCR_CELL_HEIGHT;
volatile uint8_t framebuffer[SCR_CELL_WIDTH * SCR_CELL_HEIGHT] = {0};
volatile uint32_t lastRandomTurnMicros = 0;
uint32_t randomTurnDelays[] = { RANDOM_FLIP_MICROS, RANDOM_FLIP_MICROS / 2, RANDOM_FLIP_MICROS / 4 };
int randomTurnThresholds[] = { 600, 750, 1000 };

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_ILI_RS, OUTPUT);
  digitalWrite(PIN_ILI_RS, HIGH);
  
  pinMode(PIN_ILI_WR, OUTPUT);
  digitalWrite(PIN_ILI_WR, HIGH);

  pinMode(PIN_ILI_RD, OUTPUT);
  digitalWrite(PIN_ILI_RD, HIGH);

  pinMode(PIN_ILI_RST, OUTPUT);
  digitalWrite(PIN_ILI_RST, HIGH);

  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  
  setDataBusInput();

  resetILI();

  setupILI();

  lastRandomTurnMicros = micros();
}

void setupILI()
{
  // wait 120ms after reset before sending sleep out (as per datasheet)
  delay(120);
  sendILICommand(ILI_CMD_SLEEPOUT);

  // wait 60 ms after sending sleepout before sending DISPLAYON (as per datasheet)
  delay(60);
  sendILICommand(ILI_CMD_DISPLAYON);
  setDataBusInput(); 

  // set brightness control on
  sendILICommand(ILI_CMD_WRITECTRLDISPLAY);
  writeILIByte(ILI_MASK_BCTRL | ILI_MASK_DD | ILI_MASK_BL);
  
  // set brightness to 0xff
  sendILICommand(ILI_CMD_WRITEDISPLAYBRIGHTNESS);
  writeILIByte(0xff);

  sendILICommand(ILI_CMD_MADCTL);
  writeILIByte(ILI_MASK_MV | ILI_MASK_MY | ILI_MASK_MX);

  for (int y = 0; y < SCR_CELL_HEIGHT; y++)
  {
    for (int x = 0; x < SCR_CELL_WIDTH; x++)
    {
      framebuffer[SCR_CELL_WIDTH*y+x] = x < SCR_CELL_WIDTH / 2 ? 0xDB : 0xff;
    }
  } 

/*  uint8_t index = 0;
  for (int y = 0; y < SCR_CELL_HEIGHT; y++)
  {
    for (int x = 0; x < SCR_CELL_WIDTH; x++, index++)
    {
      framebuffer[SCR_CELL_WIDTH*y+x] = index;
    }
  } */
  writeFrameBuffer();  
}

void writeFrameBuffer()
{
  uint16_t offset = 0;
  uint16_t sy = 0;
  for (uint8_t cy = 0; cy < SCR_CELL_HEIGHT; cy++, sy += CELL_HEIGHT)
  {
    uint16_t sx = 0;
    for (uint8_t cx = 0; cx < SCR_CELL_WIDTH; cx++, offset++, sx += CELL_WIDTH)
    {
      writeCharToScreen(cx, cy, framebuffer[offset]);
    }
  }
}

void sendColumnAddress(int columnStart, int columnEnd)
{
  sendAddress(ILI_CMD_CASET, columnStart, columnEnd);
}

void sendPageAddress(int pageStart, int pageEnd)
{
  sendAddress(ILI_CMD_PASET, pageStart, pageEnd);
}

void sendAddress(uint8_t command, int addressStart, int addressEnd)
{
  sendILICommand(command);
  writeILIByte(addressStart >> 8);
  writeILIByte((uint8_t) addressStart);
  writeILIByte(addressEnd >> 8);
  writeILIByte((uint8_t) addressEnd);    
}

void setDataBusState(uint8_t state)
{
  DDRD = (state == OUTPUT) ? (DDRD | 0b00001111) : (DDRD & 0b11110000);
  DDRF = (state == OUTPUT) ? (DDRF | 0b11110000) : (DDRF & 0b00001111);
/*  for (int i = 0; i < 8; i++)
  {
      pinMode(databusPins[i], state);
  } */
}

void setDataBusInput()
{
  setDataBusState(INPUT);
}

void setDataBusOutput()
{
  setDataBusState(OUTPUT);
}

void sendILICommand(uint8_t regAddress)
{
  setILICtrlHigh();
  setILIRs(LOW);
  setDataBus(regAddress);
  toggleILIWr();
  setILIRs(HIGH);
}

uint8_t readILIByte()
{
  setDataBusInput();
  setILICtrlHigh();
  setILIRd(LOW);
  uint8_t byteRead = readDataBus();
  setILIRd(HIGH);

  return byteRead;
}

void writeILIByte(uint8_t byteWrite)
{
  setILICtrlHigh();
  setDataBus(byteWrite);
  toggleILIWr();
}

void setILICtrlHigh()
{
  setILIRs(HIGH);
  setILIRd(HIGH);
  setILIWr(HIGH);
}

void setILIRs(uint8_t state)
{
//  digitalWrite(PIN_ILI_RS, state);
  PORTE = (state == LOW) ? (PORTE & ~bit(PORTE6)) : (PORTE | bit(PORTE6));
}

void setILIRd(uint8_t state)
{
//  digitalWrite(PIN_ILI_RD, state);
  PORTD = (state == LOW) ? (PORTD & ~bit(PORTD7)) : (PORTD | bit(PORTD7));
}

void setILIWr(uint8_t state)
{
//  digitalWrite(PIN_ILI_WR, state);
  PORTC = (state == LOW) ? (PORTC & ~bit(PORTC6)) : (PORTC | bit(PORTC6));
}

void toggleILIWr()
{
  setILIWr(LOW);
  setILIWr(HIGH);
}

void resetILI()
{
  digitalWrite(PIN_ILI_RST, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_ILI_RST, HIGH);
}

void setDataBus(uint8_t data)
{
  setDataBusOutput();
  PORTF = ((PORTF & 0b11110000) ^ PORTF) | (data << 4);
  PORTD = ((PORTD & 0b00001111) ^ PORTD) | (data >> 4);
/*  for (int i = 0; i < 8; i++)
  {
    digitalWrite(databusPins[i], (data & 1));
    data = data >> 1;
  } */
}

uint8_t readDataBus()
{
  setDataBusInput();
  uint8_t data = 0;
  for (int i = 0; i < 8; i++)
  {
    data |= (digitalRead(databusPins[i]) << i);
  }
}

uint8_t displayInverted = 0;
uint8_t lastButtonState[2] = {1};
uint8_t currentButtonState[2] = {1};

void turn_white(int index)
{
  int framebufferOffset = 0;
  int count = 0;
  for (uint8_t cy = 0; cy < SCR_CELL_HEIGHT; cy++)
  {
    for (uint8_t cx = 0; cx < SCR_CELL_WIDTH; cx++, framebufferOffset++)
    {
      if (framebuffer[framebufferOffset] == 0xff)
      {
        if (count == index)
        {
          framebuffer[framebufferOffset] = 0xdb;
          whites++;
          blacks--;
          sendColumnAddress(CELL_WIDTH*cx, CELL_WIDTH*cx+CELL_WIDTH-1);
          sendPageAddress(CELL_HEIGHT*cy, CELL_HEIGHT*cy+CELL_HEIGHT-1);
          sendILICommand(ILI_CMD_MEMORYWRITE);
          for (uint8_t pxl = 0; pxl < CELL_WIDTH * CELL_HEIGHT; pxl++)
          {
            writeILIByte(0xff);
            writeILIByte(0xff);
            writeILIByte(0xff);
          }
          return;
        }
        count++;
      }
    }
  }
}

void turn_black(int index)
{
  int framebufferOffset = 0;
  int count = 0;
  for (uint8_t cy = 0; cy < SCR_CELL_HEIGHT; cy++)
  {
    for (uint8_t cx = 0; cx < SCR_CELL_WIDTH; cx++, framebufferOffset++)
    {
      if (framebuffer[framebufferOffset] == 0xdb)
      {
        if (count == index)
        {
          framebuffer[framebufferOffset] = 0xff;
          blacks++;
          whites--;
          sendColumnAddress(CELL_WIDTH*cx, CELL_WIDTH*cx+CELL_WIDTH-1);
          sendPageAddress(CELL_HEIGHT*cy, CELL_HEIGHT*cy+CELL_HEIGHT-1);
          sendILICommand(ILI_CMD_MEMORYWRITE);
          for (uint8_t pxl = 0; pxl < CELL_WIDTH * CELL_HEIGHT; pxl++)
          {
            writeILIByte(0);
            writeILIByte(0);
            writeILIByte(0);
          }
          return;
        }
        count++;
      }
    }
  }
}

uint32_t getRandomTurnDelay()
{
  int maxCols = max(whites, blacks);
  uint32_t turnDelay = randomTurnDelays[0];
  
  if (maxCols > randomTurnThresholds[1])
    turnDelay = randomTurnDelays[1];
    
  if (maxCols > randomTurnThresholds[2])
    turnDelay = randomTurnDelays[2];

  return turnDelay;
}

void writeCharToScreen(uint8_t cx, uint8_t cy, char character)
{
  uint16_t sx = cx << 3;
  uint16_t sy = cy << 3;
  
  sendColumnAddress(sx, sx+CELL_WIDTH-1);
  sendPageAddress(sy, sy+CELL_HEIGHT-1);
  sendILICommand(ILI_CMD_MEMORYWRITE);
  void *charOffset = &chargen[((uint8_t) character) << 3];
  
  for (register uint8_t row = 0; row < CELL_HEIGHT; row++, charOffset++)
  {
    register uint8_t charByte = pgm_read_byte(charOffset);
    for (register uint8_t bitIndex = 0; bitIndex < 8; bitIndex++)
    {
      register uint8_t outputByte = ((charByte & 0b10000000) == 0) ? 0 : 0xff;
      writeILIByte(outputByte);
      writeILIByte(outputByte);
      writeILIByte(outputByte);
      charByte = charByte << 1;
    }
  }  
}

void writeStringToScreen(uint8_t x, uint8_t y, char *str)
{
  for (uint8_t i = 0; i < strlen(str); i++, x++)
  {
    writeCharToScreen(x, y, str[i]);
  }
}

bool gameOver = false;

int fullCount = (SCR_CELL_WIDTH * SCR_CELL_HEIGHT);
void loop() {
  if (gameOver) return;
  
  uint32_t currentMicros = micros();
  uint32_t randomTurnDelay = getRandomTurnDelay();
  if ((currentMicros - lastRandomTurnMicros) > randomTurnDelay)
  {
    float value = blacks / (float) fullCount;
    float randVal = random(0, fullCount) / (float) (fullCount);
    if (randVal > value) turn_white(random(0, blacks));
    else turn_black(random(0, whites));
    
    lastRandomTurnMicros = currentMicros;
  } 

  if (blacks == 1200 || whites == 1200)
  {
    char secondsSurvivedStr[27] = {0};
    sprintf(secondsSurvivedStr, "You survived %d seconds!", millis() / 1000);
    
    gameOver = true;
    writeStringToScreen(SCR_CELL_WIDTH / 2 - 5, 13, "Game Over");
    writeStringToScreen(SCR_CELL_WIDTH / 2 - strlen(secondsSurvivedStr) / 2, 15, secondsSurvivedStr);
    return; 
  }
  
  currentButtonState[0] = digitalRead(PIN_BUTTON1);
  currentButtonState[1] = digitalRead(PIN_BUTTON2);
  
  if (currentButtonState[0] == LOW)
  {
    turn_black(random(0, whites));
  }
  if (currentButtonState[1] == LOW)
  {
    turn_white(random(0, blacks));
  }
      
  lastButtonState[0] = currentButtonState[0];
  lastButtonState[1] = currentButtonState[1];
}
