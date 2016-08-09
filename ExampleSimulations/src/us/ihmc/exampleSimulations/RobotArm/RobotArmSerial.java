package us.ihmc.exampleSimulations.RobotArm;
import processing.serial.*;
Serial myPort;

public class RobotArmSerial
{
   int BAUD_RATE = 115200;
   int SERVO_NUM = 7;

   double[] posD = new double[SERVO_NUM];
   int[] force = new int[SERVO_NUM];


   boolean isAllConverge = false;

   void setup()
   {
      // Open Serial Port: Your should change PORT_ID accoading to
      // your own situation.
      // Please refer to: https://www.processing.org/reference/libraries/serial/Serial.html
      int PORT_ID = 3;
      myPort = new Serial(this, Serial.list()[PORT_ID], BAUD_RATE);

      // Delay 2 seconds to wait 7Bot waking up
      delay(2000);

      ////////////////////////////////////////////////////////////////////////
      // 1- change forece status
      setForceStatus(0);  // change forece status to forceless
      delay(5000);

      setForceStatus(1);  // change forece status to normal servo
      delay(2000);

      setForceStatus(2);  // change forece status to protection
      delay(5000);

      ////////////////////////////////////////////////////////////////////////
      // 2- speed & pose setting
      setForceStatus(1);
      delay(2000);   // reboot 7Bot if previous status is not normal servo
      // To make motion much more stable, highly recommend you use fluency all the time.
      boolean[] fluentEnables = {true, true, true, true, true, true, true};
      int[] speeds_1 = {50, 50, 50, 200, 200, 200, 200};
      setSpeed(fluentEnables, speeds_1); // set speed

      float[] angles_1 = {45, 115, 65, 90, 90, 90, 80};
      setServoAngles(angles_1);  // set pose
      while (!isAllConverge)
      {
         delay(200);
      }  // wait motion converge
      float[] angles_2 = {135, 115, 65, 90, 90, 90, 80};
      setServoAngles(angles_2);
      while (!isAllConverge)
      {
         delay(200);
      }

      int[] speeds_2 = {150, 150, 150, 200, 200, 200, 200};
      setSpeed(fluentEnables, speeds_2);  // change speed

      float[] angles_3 = {45, 135, 65, 90, 90, 90, 80};
      setServoAngles(angles_3);
      while (!isAllConverge)
      {
         delay(200);
      }
      float[] angles_4 = {135, 135, 65, 90, 90, 90, 80};
      setServoAngles(angles_4);
      while (!isAllConverge)
      {
         delay(200);
      }

      ////////////////////////////////////////////////////////////////////////
      // 3- IK setting
      PVector j6 = new PVector(-100, 250, 50);
      PVector vec56 = new PVector(0, 0, -1);
      PVector vec67 = new PVector(1, 0, 0);
      float theta6 = 10;
      setIK(j6, vec56, vec67, theta6);
      delay(1500);
      //
      j6 = new PVector(0, 250, 150);
      vec56 = new PVector(0, 1, 0);
      vec67 = new PVector(1, 0, 0);
      theta6 = 55;
      setIK(j6, vec56, vec67, theta6);
      delay(1500);

      ////////////////////////////////////////////////////////////////////////
      // 4- recevie and print pose
      setForceStatus(2);
      for (int i = 0; i < 30; i++)
      {
         delay(300);
         println("Detect Poses: ", posD[0], posD[1], posD[2], posD[3], posD[4], posD[5], posD[6]);
      }

      ////////////////////////////////////////////////////////////////////////
      // 5- recevie and print force
      setForceStatus(1);
      delay(2000);
      for (int i = 0; i < 60; i++)
      {
         delay(300);
         println("Detect Forces: ", force[0], force[1], force[2], force[3], force[4], force[5], force[6]);
      }

      // Stop with protection mode
      setForceStatus(2);
      println("The End");
   }

   void draw()
   {

   }

   ////////////////////////////////////////////////////////////////////////////////////////////
/* SENT DATA TO 7BOT */

   // set motor force status: 0-forceless, 1-normal servo, 2-protection
   void setForceStatus(int status)
   {
      myPort.write(0xFE);
      myPort.write(0xF5);
      myPort.write(status & 0x7F);
   }

   // set motion fluency & speeds (0~250 ---> 0~25)
   void setSpeed(boolean fluentEnables[], int speeds[])
   {
      // 1- Process Data
      int[] sendData = new int[SERVO_NUM];
      for (int i = 0; i < SERVO_NUM; i++)
      {
         sendData[i] = constrain(speeds[i], 0, 250) / 10;
         if (fluentEnables[i])
            sendData[i] += 64;
      }
      // 2- Send Data
      myPort.write(0xFE);
      myPort.write(0xF7);
      for (int i = 0; i < SERVO_NUM; i++)
      {
         myPort.write(sendData[i] & 0x7F);
      }
   }

   // set Servo angles
   void setServoAngles(float servoAngles[])
   {
      isAllConverge = false;
      // 1- Process Data
      int[] sendData = new int[SERVO_NUM];
      for (int i = 0; i < SERVO_NUM; i++)
      {
         sendData[i] = (int) (servoAngles[i] * 50 / 9);
      }
      // 2- Send Data
      myPort.write(0xFE);
      myPort.write(0xF9);
      for (int i = 0; i < SERVO_NUM; i++)
      {
         myPort.write((sendData[i] / 128) & 0x7F);
         myPort.write(sendData[i] & 0x7F);
      }
   }

   // IK6(6 angles)
   // j6:mm(-500~500), vec:(-1.0~1.0)--->(-500~500), theta:Degrees
   void setIK(PVector j6, PVector vec56, PVector vec67, float theta6)
   {
      isAllConverge = false;
      // 1- Process Data
      PVector j6_c = new PVector(constrain(j6.x, -500, 500), constrain(j6.y, -500, 500), constrain(j6.z, -500, 500));
      PVector vec56_c = vec56;
      vec56_c.normalize();
      vec56_c.mult(500);
      PVector vec67_c = vec67;
      vec67_c.normalize();
      vec67_c.mult(500);
      //
      int[] sendData = new int[10];
      sendData[0] = (int) abs(j6_c.x);
      if (j6_c.x < 0)
         sendData[0] += 1024;
      sendData[1] = (int) abs(j6_c.y);
      if (j6_c.y < 0)
         sendData[1] += 1024;
      sendData[2] = (int) abs(j6_c.z);
      if (j6_c.z < 0)
         sendData[2] += 1024;
      //
      sendData[3] = (int) abs(vec56_c.x);
      if (vec56_c.x < 0)
         sendData[3] += 1024;
      sendData[4] = (int) abs(vec56_c.y);
      if (vec56_c.y < 0)
         sendData[4] += 1024;
      sendData[5] = (int) abs(vec56_c.z);
      if (vec56_c.z < 0)
         sendData[5] += 1024;
      //
      sendData[6] = (int) abs(vec67_c.x);
      if (vec67_c.x < 0)
         sendData[6] += 1024;
      sendData[7] = (int) abs(vec67_c.y);
      if (vec67_c.y < 0)
         sendData[7] += 1024;
      sendData[8] = (int) abs(vec67_c.z);
      if (vec67_c.z < 0)
         sendData[8] += 1024;
      //
      sendData[9] = (int) (theta6 * 50 / 9);
      // 2- Send Data
      myPort.write(0xFE);
      myPort.write(0xFA);
      for (int i = 0; i < 10; i++)
      {
         myPort.write((sendData[i] / 128) & 0x7F);
         myPort.write(sendData[i] & 0x7F);
      }
   }

   // IK5(5 angles)
   // j6:mm(-500~500), vec:(-1.0~1.0)--->(-500~500),  theta:Degrees
   void setIK(PVector j6, PVector vec56, float theta5, float theta6)
   {
      isAllConverge = false;
      // 1- Process Data
      PVector j6_c = new PVector(constrain(j6.x, -500, 500), constrain(j6.y, -500, 500), constrain(j6.z, -500, 500));
      PVector vec56_c = vec56;
      vec56_c.normalize();
      vec56_c.mult(500);
      //
      int[] sendData = new int[8];
      sendData[0] = (int) abs(j6_c.x);
      if (j6_c.x < 0)
         sendData[0] += 1024;
      sendData[1] = (int) abs(j6_c.y);
      if (j6_c.y < 0)
         sendData[1] += 1024;
      sendData[2] = (int) abs(j6_c.z);
      if (j6_c.z < 0)
         sendData[2] += 1024;
      //
      sendData[3] = (int) abs(vec56_c.x);
      if (vec56_c.x < 0)
         sendData[3] += 1024;
      sendData[4] = (int) abs(vec56_c.y);
      if (vec56_c.y < 0)
         sendData[4] += 1024;
      sendData[5] = (int) abs(vec56_c.z);
      if (vec56_c.z < 0)
         sendData[5] += 1024;
      //
      sendData[6] = (int) (theta5 * 50 / 9);
      sendData[7] = (int) (theta6 * 50 / 9);
      // 2- Send Data
      myPort.write(0xFE);
      myPort.write(0xFB);
      for (int i = 0; i < 8; i++)
      {
         myPort.write((sendData[i] / 128) & 0x7F);
         myPort.write(sendData[i] & 0x7F);
      }
   }

   // IK3(3 angles)
   // j5:mm(-500~500),   theta:Degrees
   void setIK(PVector j5, float theta3, float theta4, float theta5, float theta6)
   {
      isAllConverge = false;
      // 1- Process Data
      PVector j5_c = new PVector(constrain(j5.x, -500, 500), constrain(j5.y, -500, 500), constrain(j5.z, -500, 500));
      //
      int[] sendData = new int[7];
      sendData[0] = (int) abs(j5_c.x);
      if (j5_c.x < 0)
         sendData[0] += 1024;
      sendData[1] = (int) abs(j5_c.y);
      if (j5_c.y < 0)
         sendData[1] += 1024;
      sendData[2] = (int) abs(j5_c.z);
      if (j5_c.z < 0)
         sendData[2] += 1024;
      //
      sendData[3] = (int) (theta3 * 50 / 9);
      sendData[4] = (int) (theta4 * 50 / 9);
      sendData[5] = (int) (theta5 * 50 / 9);
      sendData[6] = (int) (theta6 * 50 / 9);
      // 2- Send Data
      myPort.write(0xFE);
      myPort.write(0xFC);
      for (int i = 0; i < 7; i++)
      {
         myPort.write((sendData[i] / 128) & 0x7F);
         myPort.write(sendData[i] & 0x7F);
      }
   }

   ////////////////////////////////////////////////////////////////////////////////////////////
/* RECEIVE DATA FROM 7BOT */

   int[] dataBuf = new int[60];
   boolean beginFlag = false;
   int instruction = 0;
   int cnt = 0;

   void serialEvent(Serial myPort)
   {

      while (myPort.available() > 0)
      {

         // read data
         int rxBuf = myPort.read();
         if (!beginFlag)
         {
            beginFlag = rxBuf == 0xFE ? true : false; // Beginning Flag 0xFE
         }
         else
         {
            if (instruction == 0)
               instruction = rxBuf - 240;
            else
            {
               switch (instruction)
               {

               case 9:
                  dataBuf[cnt++] = rxBuf;
                  if (cnt >= SERVO_NUM * 2 + 1)
                  {
                     beginFlag = false;
                     instruction = 0;
                     cnt = 0;
                     for (int i = 0; i < SERVO_NUM; i++)
                     {
                        int posCode = dataBuf[i * 2] * 128 + dataBuf[i * 2 + 1];
                        force[i] = posCode % 16384 / 1024;
                        if (posCode / 16384 > 0)
                           force[i] = -force[i];

                        posD[i] = (posCode % 1024) * 9 / 50; // convert 0~1000 code to 0~180 degree(accuracy 0.18 degree)
                     }

                     if (dataBuf[(SERVO_NUM - 1) * 2 + 2] == 1)
                        isAllConverge = true;
                     else
                        isAllConverge = false;
                  }
                  break;

               default:
                  beginFlag = false;
                  instruction = 0;
                  cnt = 0;
                  break;
               }
            }
         }
      }
   }
}
