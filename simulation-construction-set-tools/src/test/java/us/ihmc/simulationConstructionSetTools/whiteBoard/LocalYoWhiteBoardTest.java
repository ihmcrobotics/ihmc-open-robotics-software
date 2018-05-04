package us.ihmc.simulationConstructionSetTools.whiteBoard;


import java.io.IOException;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LocalYoWhiteBoardTest extends YoWhiteBoardTest
{

	@ContinuousIntegrationTest(estimatedDuration = 1.8)
	@Test(timeout=300000)
   public void testLocalYoWhiteBoardOne() throws IOException
   {
      LocalYoWhiteBoard leftWhiteBoard = new LocalYoWhiteBoard("left", new YoVariableRegistry("leftRegistry"));
      LocalYoWhiteBoard rightWhiteBoard = new LocalYoWhiteBoard("right", new YoVariableRegistry("rightRegistry"));

      leftWhiteBoard.setMyBrotherWhiteBoard(rightWhiteBoard);

      int numberOfTests = 2000;
      doASynchronizedWriteThenReadTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1000, 2000);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.8)
	@Test(timeout=300000)
   public void testLocalYoWhiteBoardTwo() throws IOException
   {
      LocalYoWhiteBoard leftWhiteBoard = new LocalYoWhiteBoard("left", new YoVariableRegistry("leftRegistry"));
      LocalYoWhiteBoard rightWhiteBoard = new LocalYoWhiteBoard("right", new YoVariableRegistry("rightRegistry"));

      leftWhiteBoard.setMyBrotherWhiteBoard(rightWhiteBoard);

      int numberOfTests = 1000;
      doAnAsynchronousTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 250, 500);
   }
}
