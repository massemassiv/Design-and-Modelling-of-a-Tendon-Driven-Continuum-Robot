#include <Arduino.h>
#include <Servo.h>
#include "setup_sensor.h"
#include "order.h"
#include "slave.h"
#include "parameters.h"
#include "setup.h"
bool update_steppers=false;
bool is_connected = false; ///< True if the connection with the master is available
//int8_t motor_speed = 0;
//int16_t servo_angle = INITIAL_THETA;
//Servo servomotor;

void update_motor_orders();

void setup()
{
  // Init Serial
  Serial.begin(SERIAL_BAUD);

  setup_motors(); // För att konfigurera motorerna
  //setup_sensor();
  // Init Motor
  //pinMode(MOTOR_PIN, OUTPUT);
  //pinMode(DIRECTION_PIN, OUTPUT);
  // Stop the car
  //stop();

  // Init Servo
  //servomotor.attach(SERVOMOTOR_PIN);
  // Order between 0 and 180
  //servomotor.write(INITIAL_THETA);

  // Wait until the arduino is connected to master

  while(!is_connected)
  {
    write_order(HELLO);
    wait_for_bytes(1, 1000);
    get_messages_from_serial();
    stepper1.move(4);
    stepper1.move(-4);
    stepper2.move(4);
    stepper2.move(-4);
    stepper3.move(4);
    stepper3.move(-4);
  }

}
void update_motor_orders2();
void loop()
{

  get_messages_from_serial();
  delay(10);
  /*if (current_motor_idx==0) {
    update_motor_orders2();
  
  }*/
 
  //update_motor_orders2();
  
  if(update_steppers){
    update_motor_orders2();
  };

}

void update_motor_orders2(){ 
    int done1=0;
    int done2=0;
    int done3=0;
    //stepper2.move(100);
    //stepper2.move(-100);
    //s_degrees[0]=s_degrees[0]-(s_degrees[0]/1.8);
    while(done1+done2+done3!=3){
      if(abs(s_degrees[0]-1)>1){
        
        if(s_degrees[0]>0.0)
        {
          stepper1.rotate(2);
          //s_degrees[0]-=1.8;
          s_degrees[0]=floor(s_degrees[0]-2);
        }
        else
        {
          stepper1.rotate(-2);
          //s_degrees[0]+=1.8;
          s_degrees[0]=floor(s_degrees[0]+2);
          };
        
      }
      else
      {
        done1=1;
        
      };
      delay(1);
      if(abs(s_degrees[1]-1)>1){
        
        if(s_degrees[1]>0.0)
        {
          stepper2.rotate(2);
          //s_degrees[0]-=1.8;
          s_degrees[1]=floor(s_degrees[1]-2);
        }
        else
        {
          stepper2.rotate(-2);
          //s_degrees[0]+=1.8;
          s_degrees[1]=floor(s_degrees[1]+2);
          };
          
      }
      else
      {
        done2=1;
        
      };
      delay(1);
      
      if(abs(s_degrees[2]-1)>1){
        
        if(s_degrees[2]>0.0)
        {
          stepper3.rotate(2);
          //s_degrees[0]-=1.8;
          s_degrees[2]=floor(s_degrees[2]-2);
        }
        else
        {
          stepper3.rotate(-2);
          //s_degrees[0]+=1.8;
          s_degrees[2]=floor(s_degrees[2]+2);
          };
          
      }
      else
      {
        done3=1;
        
      };
      delay(1);
      
    };
    update_steppers=false;
}

void wait_for_received()
{
  Order rec_order=read_order();
  while (rec_order!=RECEIVED) {
  rec_order=read_order();
    /*if(rec_order==HELLO or rec_order==ALREADY_CONNECTED)
    {
      write_order(ALREADY_CONNECTED);
    }*/
  };
}

void wait_for_order(Order wait_order)
{
  Order rec_order=read_order();
  while (rec_order!=wait_order) {
  rec_order=read_order();
    /*if(rec_order==HELLO or rec_order==ALREADY_CONNECTED)
    {
      write_order(ALREADY_CONNECTED);
    }*/
  };
}


void get_messages_from_serial()
{
  if(Serial.available() > 0)
  {
    // The first byte received is the instruction
    Order order_received = read_order();

    //write_order(order_received); // Vi skickar rätt saker 

    if(order_received == HELLO)
    {
      // If the cards haven't say hello, check the connection
      if(is_connected == false)
      {
        is_connected = true;
        write_order(HELLO); // Körs ej
      }
      else
      {
        // If we are already connected do not send "hello" to avoid infinite loop
        write_order(HELLO); // körs ej
      }
    }
    else if(order_received == ALREADY_CONNECTED)
    {
      is_connected = true;
    }
    else
    {
      switch(order_received)
      {
        case STOP:
        {
          //motor_speed = 0;
          //stop();
          if(DEBUG)
          {
            //write_order(STOP);
          }
          break;
        }
        case SERVO:
        {
          /*
          int itendon_dist = read_i32();
          servo_angle=float(itendon_dist);//*SERVO_CONV;//SERVO_CONV*itendon_dist;
          //servo_angle=dist2deg(servo_angle);
          s_degrees[current_motor_idx]= -servo_angle;
          current_motor_idx+=1;
          current_motor_idx =current_motor_idx % 3;
          */
          write_order(RECEIVED);
          for(int i=0;i<=2;i++)
          {
            
            wait_for_order(static_cast<Order>(i+14));
            int itendon_dist=read_i32();
            servo_angle=float(itendon_dist);
            
            s_degrees[i]=-servo_angle;
            if(DEBUG){
              write_order(static_cast<Order>(i+14));
              write_i32(s_degrees[i]);
            }
            write_order(static_cast<Order>(i+17));
            
          }
          update_steppers=true;
          
          if(DEBUG)
          {
            write_order(SERVO);
            write_i32(servo_angle);
          }
          break;
        }
        case MOTOR:
        {
          // between -100 and 100
          //motor_speed = read_i8();
          //send_sensor_data=true;
          if(DEBUG)
          {
            //write_order(MOTOR);
            //write_i16(motor_speed);
          }
          break;
        }
        case GET_SENSOR:
        {
          // between -100 and 100
          //motor_speed = read_i8();
          //send_sensor_data=true;
          if(DEBUG)
          {
            //write_order(MOTOR);
            //write_i16(motor_speed);
          }
          break;
        }
  			// Unknown order
  			default:
          write_order(ERROR);
          write_i16(404);
  				return;
      }
    }
    
    write_order(RECEIVED); // Confirm the reception, funkar att ändra denna
  }
}


Order read_order()
{
	return (Order) Serial.read();
}

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
	unsigned long startTime = millis();
	//Wait for incoming bytes or exit if timeout
	while ((Serial.available() < num_bytes) && (millis() - startTime < timeout)){}
}

// NOTE : Serial.readBytes is SLOW
// this one is much faster, but has no timeout
void read_signed_bytes(int8_t* buffer, size_t n)
{
	size_t i = 0;
	int c;
	while (i < n)
	{
		c = Serial.read();
		if (c < 0) break;
		*buffer++ = (int8_t) c; // buffer[i] = (int8_t)c;
		i++;
	}
}

int8_t read_i8()
{
	wait_for_bytes(1, 100); // Wait for 1 byte with a timeout of 100 ms
  return (int8_t) Serial.read();
}

int16_t read_i16()
{
  int8_t buffer[2];
	wait_for_bytes(2, 100); // Wait for 2 bytes with a timeout of 100 ms
	read_signed_bytes(buffer, 2);
  return (((int16_t) buffer[0]) & 0xff) | (((int16_t) buffer[1]) << 8 & 0xff00);
}

int32_t read_i32()
{
  int8_t buffer[4];
	wait_for_bytes(4, 200); // Wait for 4 bytes with a timeout of 200 ms
	read_signed_bytes(buffer, 4);
  return (((int32_t) buffer[0]) & 0xff) | (((int32_t) buffer[1]) << 8 & 0xff00) | (((int32_t) buffer[2]) << 16 & 0xff0000) | (((int32_t) buffer[3]) << 24 & 0xff000000);
}

void write_order(enum Order myOrder)
{
	uint8_t* Order = (uint8_t*) &myOrder;
  Serial.write(Order, sizeof(uint8_t));
}

void write_i8(int8_t num)
{
  Serial.write(num);
}

void write_i16(int16_t num)
{
	int8_t buffer[2] = {(int8_t) (num & 0xff), (int8_t) (num >> 8)};
  Serial.write((uint8_t*)&buffer, 2*sizeof(int8_t));
}

void write_i32(int32_t num)
{
	int8_t buffer[4] = {(int8_t) (num & 0xff), (int8_t) (num >> 8 & 0xff), (int8_t) (num >> 16 & 0xff), (int8_t) (num >> 24 & 0xff)};
  Serial.write((uint8_t*)&buffer, 4*sizeof(int8_t));
}
