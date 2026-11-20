package us.ihmc.perception.gpuMapping;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Runs the global map outlier filter on hand-built maps: a measured floor at height 0 with things on it. Uses the default
 * parameters, so 2 cm cells, raised surfaces kept from a 4 x 4 cell square up, and 3 cm of height tolerance.
 */
public class HeightMapOutlierFilterTest
{
   private static final float FLOOR_HEIGHT = 0.0f;
   private static final float MEASURED_VARIANCE = 0.01f;
   private static final float UNSEEN_VARIANCE = -1.0f;
   /** What removed cells are reset to, distinct from the floor so a removal is visible in the mean too. */
   private static final float RESET_HEIGHT = -0.05f;

   private HeightMapExtractor extractor;
   private int cellsPerAxis;
   private Mat mean;
   private Mat variance;
   private FloatIndexer meanIndexer;
   private FloatIndexer varianceIndexer;

   @BeforeEach
   public void setUp()
   {
      HeightMapParameters parameters = new HeightMapParameters();
      assertEquals(0.02, parameters.getCellSize(), 1e-9, "The cell counts below assume 2 cm cells");
      assertEquals(0.08, parameters.getOutlierMinSurfaceSize(), 1e-9, "The surface sizes below assume a 4 cell minimum");

      extractor = new HeightMapExtractor(parameters);
      extractor.reset(RESET_HEIGHT);
      cellsPerAxis = extractor.getHeightMap().rows();

      mean = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      variance = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      meanIndexer = mean.createIndexer();
      varianceIndexer = variance.createIndexer();
      fill(0, cellsPerAxis - 1, 0, cellsPerAxis - 1, FLOOR_HEIGHT);
   }

   @AfterEach
   public void tearDown()
   {
      meanIndexer.close();
      varianceIndexer.close();
      mean.close();
      variance.close();
      extractor.destroy();
   }

   @Test
   public void testSmallProtrusionsAreRemoved()
   {
      fill(20, 20, 20, 20, 0.2f); // A single spike
      fill(20, 22, 50, 52, 0.25f); // A 3 x 3 blob, smaller than the minimum surface
      fill(130, 130, 20, 40, 0.2f); // A strip one cell wide

      runFilter();

      assertRemoved(20, 20, 20, 20);
      assertRemoved(20, 22, 50, 52);
      assertRemoved(130, 130, 20, 40);
      assertMeasured(0, 10, 0, cellsPerAxis - 1, FLOOR_HEIGHT);
   }

   @Test
   public void testRealSurfacesAreKept()
   {
      fill(20, 23, 80, 83, 0.25f); // A 4 x 4 square, the minimum surface
      fill(60, 100, 60, 100, 0.15f); // A box, with corners
      fill(60, 100, 59, 59, 0.07f); // Cells straddling the box's edge, halfway up
      fill(20, 21, 110, 111, -0.2f); // A pit

      runFilter();

      assertMeasured(20, 23, 80, 83, 0.25f);
      assertMeasured(60, 100, 60, 100, 0.15f);
      assertMeasured(60, 100, 59, 59, 0.07f);
      assertMeasured(20, 21, 110, 111, -0.2f);
      assertMeasured(0, 10, 0, cellsPerAxis - 1, FLOOR_HEIGHT);
   }

   @Test
   public void testProtrusionWithNothingMeasuredAroundIsKept()
   {
      fill(140, 180, 140, 180, FLOOR_HEIGHT);
      setUnseen(140, 180, 140, 180);
      fill(159, 160, 159, 160, 0.2f);

      runFilter();

      assertMeasured(159, 160, 159, 160, 0.2f);
   }

   private void runFilter()
   {
      extractor.getHeightMap().upload(mean);
      extractor.getHeightMapVariance().upload(variance);
      extractor.removeOutliers();
      extractor.synchronize();
      extractor.getHeightMap().download(mean);
      extractor.getHeightMapVariance().download(variance);
      meanIndexer.close();
      varianceIndexer.close();
      meanIndexer = mean.createIndexer();
      varianceIndexer = variance.createIndexer();
   }

   private void fill(int firstRow, int lastRow, int firstColumn, int lastColumn, float height)
   {
      for (int row = firstRow; row <= lastRow; row++)
      {
         for (int column = firstColumn; column <= lastColumn; column++)
         {
            meanIndexer.put(row, column, height);
            varianceIndexer.put(row, column, MEASURED_VARIANCE);
         }
      }
   }

   private void setUnseen(int firstRow, int lastRow, int firstColumn, int lastColumn)
   {
      for (int row = firstRow; row <= lastRow; row++)
      {
         for (int column = firstColumn; column <= lastColumn; column++)
            varianceIndexer.put(row, column, UNSEEN_VARIANCE);
      }
   }

   private void assertRemoved(int firstRow, int lastRow, int firstColumn, int lastColumn)
   {
      for (int row = firstRow; row <= lastRow; row++)
      {
         for (int column = firstColumn; column <= lastColumn; column++)
         {
            assertEquals(UNSEEN_VARIANCE, varianceIndexer.get(row, column), "Cell (" + row + ", " + column + ") wasn't removed");
            assertEquals(RESET_HEIGHT, meanIndexer.get(row, column), 1e-6f, "Cell (" + row + ", " + column + ") wasn't reset");
         }
      }
   }

   private void assertMeasured(int firstRow, int lastRow, int firstColumn, int lastColumn, float height)
   {
      for (int row = firstRow; row <= lastRow; row++)
      {
         for (int column = firstColumn; column <= lastColumn; column++)
         {
            assertEquals(MEASURED_VARIANCE, varianceIndexer.get(row, column), "Cell (" + row + ", " + column + ") was removed");
            assertEquals(height, meanIndexer.get(row, column), 1e-6f, "Cell (" + row + ", " + column + ") changed");
         }
      }
   }
}
