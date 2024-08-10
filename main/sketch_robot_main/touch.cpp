#include <Arduino.h>

#define xLow  A0
#define xHigh A2
#define yLow A1
#define yHigh A3

typedef struct {
  unsigned int x;
  unsigned int y;
} POS;
#define BUFFER_LEN  4
POS buffer[BUFFER_LEN] = { {-1,-1} };

bool touch_read(int&x, int &y) {
  // put your main code here, to run repeatedly:
  pinMode(xLow,OUTPUT);
  pinMode(xHigh,OUTPUT);
  digitalWrite(xLow,LOW);
  digitalWrite(xHigh,HIGH);

  digitalWrite(yLow,LOW);
  digitalWrite(yHigh,LOW);

  pinMode(yLow,INPUT);
  pinMode(yHigh,INPUT);
  delay(10);

  //xLow has analog port -14 !!
  x=analogRead(yLow);

  pinMode(yLow,OUTPUT);
  pinMode(yHigh,OUTPUT);
  digitalWrite(yLow,LOW);
  digitalWrite(yHigh,HIGH);

  digitalWrite(xLow,LOW);
  digitalWrite(xHigh,LOW);

  pinMode(xLow,INPUT);
  pinMode(xHigh,INPUT);
  delay(10);

  //xLow has analog port -14 !!
  y=analogRead(xLow);

  if( x < 70 || y < 70 || x >760 || y > 780) {
    x = y = 0;
  }
  // Serial.print("x=");Serial.print(x);
  // Serial.print(",y=");Serial.println(y);
  // delay(100);
  // if( x == 0 && y == 0)
  //   return false;
if(1) {
  memcpy(&buffer[0], &buffer[1], sizeof(POS)*(BUFFER_LEN-1));
  buffer[BUFFER_LEN-1] = { x,y };
  int c = 0;
  POS sum = {0,0};
  for( int i=0;i<BUFFER_LEN;i++) {
    if( buffer[i].x && buffer[i].y ) {
      c++;
      sum.x += buffer[i].x;
      sum.y += buffer[i].y;
    }    
  }

  if( !c ) return false;

  x = sum.x / c;
  y = sum.y / c;
}
else {
  if( x == 0 && y == 0) return false;
}
  return true;
}