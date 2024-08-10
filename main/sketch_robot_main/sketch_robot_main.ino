#include"InverseKinematics.h"
#include <SoftwareSerial.h>
#include<math.h>
SoftwareSerial robot(5,4);
// extern void setup1();
extern void vector(double x,double y);
int theta[] = {-5,-12,0};
int readline(int readch, char *buffer, int len) {
  static int pos = 0;
  int rpos;

  if (readch > 0) {
      switch (readch) {
          case '\r': // Ignore CR
              break;
          case '\n': // Return on new-line
              rpos = pos;
              pos = 0;  // Reset position index ready for next time
              return rpos;
          default:
              if (pos < len-1) {
                  buffer[pos++] = readch;
                  buffer[pos] = 0;
              }
      }
  }
  return 0;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  robot.begin(115200);
  
  // serial_robot.println("reset");

  Serial.println("waiting robot");
  robot.println("reset");
  bool success = false;
  unsigned long tm = millis();
  while( millis() < tm + 15000 ) {
    char buf[80];
    if(readline(robot.read(), buf, 80) > 0) {
      if( strncmp(buf,"ok",2)==0) {
        success = true;
        break;
      }
    }
  }

  Serial.println(success?"robot ready!":"robot does not respond!");
  if( success ) {
    for( int i=90;i<=121;i+=1) {
      robot.println("0 "+String(i-5));
      robot.println("1 "+String(i-12));
      robot.println("2 "+String(i));
      delay(30);
    }
  }

}

extern bool touch_read(int&x, int&y);

extern double corA, corB, corC;
Machine machine(6,7.5,5.2,6.3);

double kp=0.2/30;
  double integral = 0,i_constant;
  double max,min;
  double l_deviation_x = 0,l_deviation_y = 0;
  double kd=((13.5-1)/30)*0.017;
  // double kd=0;
  bool is_d_valid_x = false,is_d_valid_y = false;
  double ki=kp/5;
  // double ki=0;
  double i_x=0, i_y=0;

  double x0=0,y0=0;

double constraints(double v, double vmin, double vmax) {
  return min(max(v,vmin),vmax);
}
double D_X, D_Y;
double x_time1 = 0,x_time2;
double dt = 0;
double xpid(double x) {
  
  double deviation = x-x0;
  // double D = 0;
  if(is_d_valid_x){
    D_X = (deviation - l_deviation_x)/dt;
  }
  else{
    is_d_valid_x = true;
    D_X=0;
  }
  l_deviation_x = deviation;

   i_x = i_x + deviation*dt; //i_x*0.7+x*0.3*dt;

  
  return constraints(deviation*kp + D_X*kd + i_x*ki,-0.25, 0.25);
}
double y_time1 = 0,y_time2 = 2;
double ypid(double y) {
  double deviation = y-y0;

  if(is_d_valid_y){
    D_Y = (deviation - l_deviation_y)/dt;
    // Serial.println(D);
  }
  else{
    is_d_valid_y = true;
    D_Y=0;
  }
  l_deviation_y = deviation;
  // i_y = i_y*0.7+y*0.3*dt;
  i_y += deviation * dt;
  return constraints(deviation*kp + D_Y*kd + i_y*ki,-0.25,0.25);
}

// void loop() {
//       robot.println("0 "+String(140+theta[0]));
//     robot.println("1 "+String(140+theta[1]));    
//     robot.println("2 "+String(140+theta[2]));
// }

double r = 3;
double angle1 =0;
double count = 0;
void loop() {
  int x,y;
  double xx,yy;

  if( touch_read(x,y)) 
  {
    yy =constraints( (y-375)*13.0/700, -7,7);
    xx =constraints( (x-405)*17.0/700,-8.5,8.5);
    Serial.println( String(i_x) + "," + String(i_y) + ","+ String(xx) + "," + String(yy) + "," + String(dt) + ","+String(D_X) + ","+String(D_Y));

    x_time2 = millis();
    if(!x_time1){
      x_time1 = x_time2-20;
    }
    dt = (x_time2 - x_time1)/1000;
  //Serial.println(time);//time == 0.02
    x_time1 = x_time2;

    xx = xpid(xx);
    yy = ypid(yy);
    //Serial.println( String(xx) + "," + String(yy));
    // Serial.println(String(D_X)+" "+String(D_Y));
    // Serial.println(String(i_x)+" "+String(i_y));
    // xx = 0;
    // yy = 0;
    double leg0 = machine.theta(0,8,-xx,-yy);
    double leg1 = machine.theta(1,8,-xx,-yy);
    double leg2 = machine.theta(2,8,-xx,-yy);

    robot.println("0 "+String(int(leg0-30+theta[0])));
    robot.println("1 "+String(int(leg1-30+theta[1])));    
    robot.println("2 "+String(int(leg2-30+theta[2])));
    // vector(xx,yy);
    //Serial.println(String(leg0) + "," + String(leg1) +","+ String(leg2));



    // integral += xx * i_constant;
    // if(integral>=max){
    //   integral = max;
    // }
    // if(integral<=min){
    //   integral = min;
    // }

    // deviation = xx;
    // D = (deviation - l_deviation) * d_constant;
    // l_deviation = deviation;


  }
  //Serial.println(String(x0) +" " + String(y0));
  static char cmd[80];
  //static int cmd_len = 0;
  if( Serial.available()) {
    if( readline(Serial.read(),cmd,80 )) {
      // cmd
      switch( cmd[0]) {
        case '0': x0=0;y0=0;
        //Serial.println("0______________________________");
                  break;
        case '1': x0 += 1;y0 += 1;
        //Serial.println("1______________________________");
        
        break;

      }
      //
    }
  }

  // count+=1;
  // if(count>=5){
  //   x0 = r*cos(angle1*3.14/180);
  //   y0 = r*sin(angle1*3.14/180);
  //   angle1 +=10;
  //   count = 0;
  //   Serial.println(angle1);
    
  // }
  
  
}
