package com.yobotics.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

public class UDPYoWhiteBoardTest extends YoWhiteBoardTest
{
   @Test
   public void testUDPWhiteBoard() throws IOException
   {
      String IPAddress = "localHost";
      int leftSendRightReceivePort = 8456;
      int leftReceiveRightSendPort = 8654;

      boolean throwOutStalePackets = false;
      
      UDPYoWhiteBoard leftWhiteBoard = new UDPYoWhiteBoard(IPAddress, leftSendRightReceivePort, leftReceiveRightSendPort, throwOutStalePackets);
      UDPYoWhiteBoard rightWhiteBoard = new UDPYoWhiteBoard(IPAddress, leftReceiveRightSendPort, leftSendRightReceivePort, throwOutStalePackets);

      Thread leftWhiteBoardThread = new Thread(leftWhiteBoard);
      Thread rightWhiteBoardThread = new Thread(rightWhiteBoard);

      leftWhiteBoardThread.start();
      rightWhiteBoardThread.start();

      int numberOfTests = 1000;
      doATest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 100, 200);
   }


}
