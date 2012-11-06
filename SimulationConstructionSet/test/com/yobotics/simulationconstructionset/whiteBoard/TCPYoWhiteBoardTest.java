package com.yobotics.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

public class TCPYoWhiteBoardTest extends YoWhiteBoardTest
{
   @Test(timeout = 10000l)
   public void testTCPWhiteBoardOne() throws IOException
   {
      String IPAddress = "localHost";
      int port = 8456;

      TCPYoWhiteBoard leftWhiteBoard = new TCPYoWhiteBoard("leftTest", port);
      TCPYoWhiteBoard rightWhiteBoard = new TCPYoWhiteBoard("rightTest", IPAddress, port);

      Thread leftWhiteBoardThread = new Thread(leftWhiteBoard);
      Thread rightWhiteBoardThread = new Thread(rightWhiteBoard);

      leftWhiteBoardThread.start();
      rightWhiteBoardThread.start();

      int numberOfTests = 1000;
      doASynchronizedWriteThenReadTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1001, 2001);
   }
   
   
   @Test(timeout = 10000l)
   public void testTCPWhiteBoardTwo() throws IOException
   {
      String IPAddress = "localHost";
      int port = 8456;

      TCPYoWhiteBoard leftWhiteBoard = new TCPYoWhiteBoard("leftTest", port);
      TCPYoWhiteBoard rightWhiteBoard = new TCPYoWhiteBoard("rightTest", IPAddress, port);

      Thread leftWhiteBoardThread = new Thread(leftWhiteBoard);
      Thread rightWhiteBoardThread = new Thread(rightWhiteBoard);

      leftWhiteBoardThread.start();
      rightWhiteBoardThread.start();

      int numberOfTests = 1000;
      doAnAsynchronousTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1000, 2000);
   }
}
