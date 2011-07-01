package com.yobotics.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

public class TCPYoWhiteBoardTest extends YoWhiteBoardTest
{
   @Test
   public void testTCPWhiteBoard() throws IOException
   {
      String IPAddress = "localHost";
      int port = 8456;

      TCPYoWhiteBoard leftWhiteBoard = new TCPYoWhiteBoard(port);
      TCPYoWhiteBoard rightWhiteBoard = new TCPYoWhiteBoard(IPAddress, port);

      Thread leftWhiteBoardThread = new Thread(leftWhiteBoard);
      Thread rightWhiteBoardThread = new Thread(rightWhiteBoard);

      leftWhiteBoardThread.start();
      rightWhiteBoardThread.start();

      int numberOfTests = 1000;
      doATest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1000, 2000);
   }
}
