package us.ihmc.simulationConstructionSetTools.whiteBoard;


import java.io.IOException;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LocalYoWhiteBoardTest extends YoWhiteBoardTest
{

	@Test
   public void testLocalYoWhiteBoardOne() throws IOException
   {
      LocalYoWhiteBoard leftWhiteBoard = new LocalYoWhiteBoard("left", new YoVariableRegistry("leftRegistry"));
      LocalYoWhiteBoard rightWhiteBoard = new LocalYoWhiteBoard("right", new YoVariableRegistry("rightRegistry"));

      leftWhiteBoard.setMyBrotherWhiteBoard(rightWhiteBoard);

      int numberOfTests = 2000;
      doASynchronizedWriteThenReadTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1000, 2000);
   }

	@Test
   public void testLocalYoWhiteBoardTwo() throws IOException
   {
      LocalYoWhiteBoard leftWhiteBoard = new LocalYoWhiteBoard("left", new YoVariableRegistry("leftRegistry"));
      LocalYoWhiteBoard rightWhiteBoard = new LocalYoWhiteBoard("right", new YoVariableRegistry("rightRegistry"));

      leftWhiteBoard.setMyBrotherWhiteBoard(rightWhiteBoard);

      int numberOfTests = 1000;
      doAnAsynchronousTest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 250, 500);
   }
}
