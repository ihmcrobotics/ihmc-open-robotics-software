package us.ihmc.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets=TestPlanTarget.Flaky)

public class TCPYoWhiteBoardTest extends YoWhiteBoardTest
{

	@DeployableTestMethod
	@Test(timeout = 300000)
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

      int numberOfTests = 500;
      doASynchronizedWriteThenReadTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 501, 1001);
   }

	@DeployableTestMethod
	@Test(timeout = 300000)
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

      int numberOfTests = 500;
      doAnAsynchronousTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 500, 1000);
   }
}
