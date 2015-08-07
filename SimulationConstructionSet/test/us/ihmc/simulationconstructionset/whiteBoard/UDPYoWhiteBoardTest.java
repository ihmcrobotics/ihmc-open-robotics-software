package us.ihmc.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

public class UDPYoWhiteBoardTest extends YoWhiteBoardTest
{

	@EstimatedDuration
	@Test(timeout=300000)
   public void testUDPWhiteBoardOne() throws IOException
   {
      String IPAddress = "localHost";
      int leftSendRightReceivePort = 8456;
      int leftReceiveRightSendPort = 8654;

      boolean throwOutStalePackets = false;

      UDPYoWhiteBoard leftWhiteBoard = new UDPYoWhiteBoard("leftTest", true, IPAddress, leftSendRightReceivePort, leftReceiveRightSendPort, throwOutStalePackets);
      UDPYoWhiteBoard rightWhiteBoard = new UDPYoWhiteBoard("rightTest", false, IPAddress, leftReceiveRightSendPort, leftSendRightReceivePort, throwOutStalePackets);

      Thread leftWhiteBoardThread = new Thread(leftWhiteBoard);
      Thread rightWhiteBoardThread = new Thread(rightWhiteBoard);

      leftWhiteBoardThread.start();

      try
      {
         Thread.sleep(2000);
      }
      catch (InterruptedException e)
      {
      }

      rightWhiteBoardThread.start();

      int numberOfTests = 500;
      doASynchronizedWriteThenReadTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 203, 207);
   }

	@EstimatedDuration
	@Test(timeout=300000)
   public void testUDPWhiteBoardTwo() throws IOException
   {
      String IPAddress = "localHost";
      int leftSendRightReceivePort = 8456;
      int leftReceiveRightSendPort = 8654;

      boolean throwOutStalePackets = false;

      UDPYoWhiteBoard leftWhiteBoard = new UDPYoWhiteBoard("leftTest", true, IPAddress, leftSendRightReceivePort, leftReceiveRightSendPort, throwOutStalePackets);
      UDPYoWhiteBoard rightWhiteBoard = new UDPYoWhiteBoard("rightTest", false, IPAddress, leftReceiveRightSendPort, leftSendRightReceivePort, throwOutStalePackets);

      Thread leftWhiteBoardThread = new Thread(leftWhiteBoard);
      Thread rightWhiteBoardThread = new Thread(rightWhiteBoard);

      leftWhiteBoardThread.start();

      try
      {
         Thread.sleep(2000);
      }
      catch (InterruptedException e)
      {
      }

      rightWhiteBoardThread.start();

      int numberOfTests = 500;
      this.doAnAsynchronousTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 234, 179);
   }


}
