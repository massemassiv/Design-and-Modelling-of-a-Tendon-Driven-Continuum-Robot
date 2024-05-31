#include <Arduino.h>
#include <Servo.h>
#include "setup_sensor.h"
#include "order.h"
#include "sensor.h"
#include "parameters.h"
//#include "setup.h"

bool is_connected = false; ///< True if the connection with the master is available
int8_t motor_speed = 0;
//int16_t servo_angle = INITIAL_THETA;
Servo servomotor;
bool send_sensor_data;
void update_motor_orders();

void setup()
{
  // Init Serial
  Serial.begin(SERIAL_BAUD);

  //Init sensor
  setup_sensor();
  

  // Wait until the arduino is connected to master
  
  while(!is_connected)
  {
    write_order(HELLO);
    wait_for_bytes(1, 1000);
    get_messages_from_serial();
  }

}

void loop()
{
  //Read serial
  
  get_messages_from_serial();

  if(send_sensor_data){
    //Send sensor data if ordered to
    write_sensor_data();
  }
  read_sensor();
  
}



void write_sensor_data()
{
for(int i=0;i<=2;i++){
  Order pos_order = static_cast<Order>(i+8);
  write_order(pos_order);
  delay(10);
  //write_i16(static_cast<int16_t>(position_vec[i]));
  write_i16((int16_t)position_vec[i]);
  delay(10);
  wait_for_received();
  }
for(int i=0;i<=2;i++){
  Order ang_order = static_cast<Order>(i+11);
  write_order(ang_order);
  delay(10);
  //write_i16(static_cast<int16_t>(angle_vec[i]));
  write_i16((int16_t)angle_vec[i]);
  delay(10);
  wait_for_received();
  
  }
  send_sensor_data=false;
};
void update_sensor_data()
{
  for(int i=0;i<=2;i++){
  write_order(MOTOR);
  delay(1);
  write_i16(static_cast<int16_t>(position_vec[i]));
  delay(1);
  wait_for_received();
  }
  for(int i=0;i<=2;i++){
  write_order(MOTOR);
  delay(1);
  write_i16(static_cast<int16_t>(angle_vec[i]));
  delay(1);
  wait_for_received();
  }
  send_sensor_data=false;
};

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
void update_position_angle(){};


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
          motor_speed = 0;
          //stop();
          if(DEBUG)
          {
            write_order(STOP);
          }
          break;
        }
        case SERVO:
        {
          delay(1);

          
          break;
        }
        case MOTOR:
        {
          // between -100 and 100
          //motor_speed = read_i8();
          send_sensor_data=true;
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
          send_sensor_data=true;
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
