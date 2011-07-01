package com.yobotics.simulationconstructionset.whiteBoard;


import java.io.IOException;

import org.junit.Test;

public class LocalYoWhiteBoardTest extends YoWhiteBoardTest
{  
   @Test
   public void testLocalYoWhiteBoard() throws IOException
   {
      LocalYoWhiteBoard leftWhiteBoard = new LocalYoWhiteBoard();
      LocalYoWhiteBoard rightWhiteBoard = new LocalYoWhiteBoard();

      leftWhiteBoard.setMyBrotherWhiteBoard(rightWhiteBoard);

      int numberOfTests = 10000;
      doATest(leftWhiteBoard, rightWhiteBoard, numberOfTests, 1000, 2000);
   }
}
