package us.ihmc.perception.heightMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.tuple3D.Point3D;

import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

/**
 * Height map indexing tools. The height map spans a square region and is parametrized by the following values:
 * - A discretization value
 * - The grid size, i.e., side length of the square region it covers
 * - a Grid center, a xy coordinate which is the middle of the grid
 * <p>
 * Cells are indexed two ways:
 * - A unique integer key, which is zero-indexed and starts in the corner of the grid which is the negative-most x and y coordinates.
 * - An (x,y) integer index pair, which is zero at the negative-most cell along each axis
 */
public class HeightMapTools
{
   /**
    * The xy-indices of the center, of the grid.
    */
   public static int computeCenterIndex(double mapSize, double cellSize)
   {
      return (int) Math.round(0.5 * mapSize / cellSize);
   }

   public static int coordinateToKey(double x, double y, double xCenter, double yCenter, double resolution, int centerIndex)
   {
      int xIndex = coordinateToIndex(x, xCenter, resolution, centerIndex);
      int yIndex = coordinateToIndex(y, yCenter, resolution, centerIndex);
      return indicesToKey(xIndex, yIndex, centerIndex);
   }

   public static double keyToXCoordinate(int key, double xCenter, double resolution, int centerIndex)
   {
      int xIndex = keyToXIndex(key, centerIndex);
      return indexToCoordinate(xIndex, xCenter, resolution, centerIndex);
   }

   public static double keyToYCoordinate(int key, double yCenter, double resolution, int centerIndex)
   {
      int yIndex = keyToYIndex(key, centerIndex);
      return indexToCoordinate(yIndex, yCenter, resolution, centerIndex);
   }

   public static int coordinateToIndex(double coordinate, double origin, double resolution)
   {
      return (int) Math.floor((coordinate - origin) / resolution);
   }

   public static int coordinateToIndex(double coordinate, double gridCenter, double resolution, int centerIndex)
   {
      return (int) Math.round((coordinate - gridCenter) / resolution) + centerIndex;
   }

   public static double indexToCoordinate(int index, double mapCenter, double resolution, int centerIndex)
   {
      return (index - centerIndex) * resolution + mapCenter;
   }

   public static int keyToXIndex(int key, int centerIndex)
   {
      return key % (2 * centerIndex + 1);
   }

   public static int keyToYIndex(int key, int centerIndex)
   {
      return key / (2 * centerIndex + 1);
   }

   public static int indicesToKey(int xIndex, int yIndex, int centerIndex)
   {
      return xIndex + yIndex * (2 * centerIndex + 1);
   }

   public static int getIndexFromCoordinates(double coordinate, float resolution, int offset)
   {
      return (int) (coordinate * resolution + offset);
   }

   public static double getCoordinateFromIndex(int index, double resolution, int offset)
   {
      return (index - offset) / resolution;
   }

   /**
    * Returns the red green and blue components of a color based on a given height.
    */
   public static double[] getRedGreenBlue(double height)
   {
      // Using interpolation between key color points
      double r, g, b;
      double magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
      double orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
      double yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
      double blueR = 0.0, blueG = 0.0, blueB = 1.0;
      double greenR = 0.0, greenG = 1.0, greenB = 0.0;
      double gradientSize = 0.2;
      double gradientLength = 1.0;
      double alpha = height % gradientLength;
      if (alpha < 0)
         alpha = 1 + alpha;
      while (alpha > 5 * gradientSize)
         alpha -= 5 * gradientSize;

      if (alpha <= gradientSize * 1)
      {
         r = InterpolationTools.linearInterpolate(magentaR, blueR, (alpha) / gradientSize);
         g = InterpolationTools.linearInterpolate(magentaG, blueG, (alpha) / gradientSize);
         b = InterpolationTools.linearInterpolate(magentaB, blueB, (alpha) / gradientSize);
      }
      else if (alpha <= gradientSize * 2)
      {
         r = InterpolationTools.linearInterpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
         g = InterpolationTools.linearInterpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
         b = InterpolationTools.linearInterpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
      }
      else if (alpha <= gradientSize * 3)
      {
         r = InterpolationTools.linearInterpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
         g = InterpolationTools.linearInterpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
         b = InterpolationTools.linearInterpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
      }
      else if (alpha <= gradientSize * 4)
      {
         r = InterpolationTools.linearInterpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
         g = InterpolationTools.linearInterpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
         b = InterpolationTools.linearInterpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
      }
      else if (alpha <= gradientSize * 5)
      {
         r = InterpolationTools.linearInterpolate(orangeR, magentaR, (alpha - gradientSize * 4) / gradientSize);
         g = InterpolationTools.linearInterpolate(orangeG, magentaG, (alpha - gradientSize * 4) / gradientSize);
         b = InterpolationTools.linearInterpolate(orangeB, magentaB, (alpha - gradientSize * 4) / gradientSize);
      }
      else
      {
         throw new RuntimeException("no valid color");
      }

      if (r == 0.0 && g == 0.0 && b == 0.0)
         throw new RuntimeException("Shouldn't return black.)");

      return new double[] {r, g, b};
   }

   public static void convertHeightMapDataToMat(Mat heightMapToPack, HeightMapData heightMapData)
   {
      int cellsPerAxis = heightMapData.getCellsPerAxis();
      int totalCells = cellsPerAxis * cellsPerAxis;

      // This is done for speed optimization
      double[] heightsAsDoubles = heightMapData.getHeights();
      float[] heightsAsFloats = new float[totalCells];

      for (int col = 0; col < cellsPerAxis; col++)
      {
         for (int row = 0; row < cellsPerAxis; row++)
         {
            // This is happening for a reason, the current implementation expects column major for the Mat objects, and the HeightMapData object is column major
            int rowMajorIndex = row * cellsPerAxis + col;
            int colMajorIndex = col * cellsPerAxis + row;

            // Get the height as for row major, and save it as column major
            float height = (float) heightsAsDoubles[rowMajorIndex];

            heightsAsFloats[colMajorIndex] = height;
         }
      }

      FloatBuffer buffer = heightMapToPack.createBuffer();
      buffer.put(heightsAsFloats);
   }

   public static void convertToHeightMapData(Mat heightMapPointer,
                                             HeightMapData heightMapDataToPack,
                                             Point3D gridCenter,
                                             float widthInMeters,
                                             float cellSizeInMeters)
   {
      widthInMeters = (float) (Math.floor(widthInMeters / cellSizeInMeters) * cellSizeInMeters);
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      heightMapDataToPack.setGridCenter(gridCenter.getX(), gridCenter.getY());

      FloatPointer floatPointer = new FloatPointer(heightMapPointer.data());
      float[] values = new float[totalCells];
      floatPointer.get(values);

      // Put height values into HeightMapData object
      for (int i = 0; i < totalCells; ++i)
      {
         float cellHeight = values[i];

         // Put it into the HeightMapData object
         int key = cellsPerAxis * (i % cellsPerAxis) + (i / cellsPerAxis);
         heightMapDataToPack.setHeight(key, cellHeight);
      }
   }

   public static float[] packArrayForFile(Mat heightMap,
                                          Point3D gridCenter,
                                          float widthInMeters,
                                          float cellSizeInMeters,
                                          HeightMapParameters heightMapParameters)
   {
      // Snap to cell resolution
      widthInMeters = (float) (Math.floor(widthInMeters / cellSizeInMeters) * cellSizeInMeters);
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Ensure the Mat is a 16-bit unsigned single channel
      if (heightMap.type() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Expected CV_16UC1 Mat");

      // Read the short values from the Mat
      ShortBuffer shortBuffer = heightMap.createBuffer();
      short[] shortHeights = new short[totalCells];
      shortBuffer.get(shortHeights);

      // Retrieve scale/offset to convert shorts → floats (real heights)
      float heightOffset = (float) heightMapParameters.getHeightOffset();
      float scaleFactor = (float) heightMapParameters.getHeightScaleFactor();

      // Prepare an output array with a header
      final int headerFloats = 6;
      float[] packedArray = new float[headerFloats + totalCells];

      // Write header
      packedArray[0] = widthInMeters;
      packedArray[1] = cellSizeInMeters;
      packedArray[2] = (float) gridCenter.getX();
      packedArray[3] = (float) gridCenter.getY();
      packedArray[4] = heightOffset;
      packedArray[5] = scaleFactor;

      // Convert shorts to floats and copy into a packed array
      for (int i = 0; i < totalCells; ++i)
      {
         packedArray[headerFloats + i] = shortHeights[i];
      }

      return packedArray;
   }
}
