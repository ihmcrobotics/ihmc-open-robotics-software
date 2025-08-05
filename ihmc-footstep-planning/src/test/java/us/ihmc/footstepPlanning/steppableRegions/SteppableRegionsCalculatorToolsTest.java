package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.steppableRegions.data.SteppableCell;
import us.ihmc.footstepPlanning.steppableRegions.data.SteppableRegionsEnvironmentModel;

import static org.junit.jupiter.api.Assertions.*;

public class SteppableRegionsCalculatorToolsTest
{
   /**
    * Basic test to make sure the {@link SteppableRegionsCalculatorTools#normalValueAsFloat(Mat, int, int)} returns the expected value.
    */
   @Test
   public void testGetNormalValueAsFloat()
   {
      int cellsPerAxis = 10;

      // We fill with zeros
      Mat dataMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1, new Scalar(0));

      // We are scaling the normal between [-1.0, 1.0], so from [0 - 255] we expect a result of -1.0
      float valueAsFloat = SteppableRegionsCalculatorTools.normalValueAsFloat(dataMat, 0, 0);
      float expectedValue = -1.0f;
      assertEquals(expectedValue, valueAsFloat);
   }

   /**
    * To set up this test we expect to have a bunch of cells that have a {@link SnapResult#VALID}.
    * All those cells will get added to the {@link SteppableRegionsEnvironmentModel} which we will use.
    * Once that setup is done, we want to see how many valid neighbors it has, we test that the
    * {@link SteppableRegionsCalculatorTools#collectCellNeighborsInEnvironment(SteppableCell, SteppableRegionsEnvironmentModel, Mat)} is working properly
    */
   @Test
   public void testFullyConnectedCell()
   {
      int cellsPerAxis = 3;
      SteppableRegionsEnvironmentModel environmentModel = new SteppableRegionsEnvironmentModel(cellsPerAxis);

      for (int x = 0; x < cellsPerAxis; x++)
      {
         for (int y = 0; y < cellsPerAxis; y++)
         {
            boolean doesntMatterForThisTest = false;
            environmentModel.addUnexpandedSteppableCell(new SteppableCell(x, y, 0.0, new Vector3D(0.0, 0.0, 0.0), cellsPerAxis, doesntMatterForThisTest));
         }
      }

      // The center cell will have a full connections list - 11111111 — all connected
      Mat connections = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
      connections.ptr(1, 1).put((byte) 0xFF);

      SteppableCell center = environmentModel.getCellAt(1, 1);
      SteppableRegionsCalculatorTools.collectCellNeighborsInEnvironment(center, environmentModel, connections);

      // Expect all 8 neighbors because the cell is surrounded
      int expectedNeighbors = 8;
      assertEquals(expectedNeighbors, center.getValidNeighbors().size());
   }

   @Test
   public void testIsConnected()
   {
      assertTrue(SteppableRegionsCalculatorTools.isConnected(3, 8));
      assertFalse(SteppableRegionsCalculatorTools.isConnected(3, 0));
      assertFalse(SteppableRegionsCalculatorTools.isConnected(8, 7));
      assertFalse(SteppableRegionsCalculatorTools.isConnected(1, 5));
   }
}
