package us.ihmc.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class LocalYoWhiteBoardTest extends YoWhiteBoardTest
{

	@EstimatedDuration
	@Test(timeout=300000)
   public void testLocalYoWhiteBoardOne() throws IOException
   {
      LocalYoWhiteBoard leftWhiteBoard = new LocalYoWhiteBoard("left", new YoVariableRegistry("leftRegistry"));
      LocalYoWhiteBoard rightWhiteBoard = new LocalYoWhiteBoard("right", new YoVariableRegistry("rightRegistry"));

      leftWhiteBoard.setMyBrotherWhiteBoard(rightWhiteBoard);

      int numberOfTests = 2000;
      doASynchronizedWriteThenReadTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1000, 2000);
   }

	@EstimatedDuration
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
