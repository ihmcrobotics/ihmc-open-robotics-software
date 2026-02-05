package us.ihmc.perception.gpuMapping;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.nio.FloatBuffer;

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

   public static int coordinateToIndex(double coordinate, double gridCenter, double resolution, int centerIndex)
   {
      return (int) Math.round((coordinate - gridCenter) / resolution) + centerIndex;
   }

   public static int coordinateToChunkIndex(double coordinate, double chunkOrigin, double resolution)
   {
      return (int) Math.floor((coordinate - chunkOrigin) / resolution);
   }

   public static double indexToCoordinate(int index, double mapCenter, double resolution, int centerIndex)
   {
      return (index - centerIndex) * resolution + mapCenter;
   }

   public static int keyToXIndex(int key, int centerIndex)
   {
      return key / (2 * centerIndex + 1);
   }

   public static int keyToYIndex(int key, int centerIndex)
   {
      return key % (2 * centerIndex + 1);
   }

   public static int indicesToKey(int xIndex, int yIndex, int centerIndex)
   {
      return yIndex + xIndex * (2 * centerIndex + 1);
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
      float[] heightsAsFloats = heightMapData.getHeights();
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

      heightMapDataToPack.setHeights(values);
   }

   public record FlattenedHeightMap(FloatPointer data, int pointCount) {}

   public static FlattenedHeightMap flattenHeightMapToXYZ(GpuMat heightMap,
                                                          double centerX,
                                                          double centerY,
                                                          int centerIndex,
                                                          float cellSize,
                                                          float invalidHeightValue)
   {
      int rows = heightMap.rows();
      int cols = heightMap.cols();

      // Download height map to CPU
      Mat cpuMap = new Mat(rows, cols, opencv_core.CV_32FC1);
      FloatPointer cpuHeights = new FloatPointer(cpuMap.data());
      heightMap.download(cpuMap);

      // Worst case: every cell is valid
      FloatPointer xyz = new FloatPointer((long) rows * cols * 3);

      int validCount = 0;

      for (int row = 0; row < rows; row++)
      {
         for (int col = 0; col < cols; col++)
         {
            float z = cpuHeights.get((long) row * cols + col);

            // Reject invalid heights
            if (Float.isNaN(z) || z == invalidHeightValue)
               continue;

            // Convert grid coordinates to world coordinates
            // Offset from center in grid cells, then convert to meters, then add center position
            float x = (float) (centerX + (col - centerIndex) * cellSize);
            float y = (float) (centerY + (row - centerIndex) * cellSize);

            int base = validCount * 3;
            xyz.put(base, x);
            xyz.put(base + 1, y);
            xyz.put(base + 2, z);

            validCount++;
         }
      }

      // Trim unused memory
      FloatPointer trimmed = new FloatPointer(validCount * 3L);
      trimmed.put(xyz.position(0).limit(validCount * 3L));

      cpuHeights.close();
      xyz.close();

      return new FlattenedHeightMap(trimmed, validCount);
   }

   public static DMatrixRMaj computeTransformSVD(FloatPointer localPoints, FloatPointer globalPoints, IntPointer correspondences, int numPoints)
   {
      // Step 1: Compute centroids
      double localCentroidX = 0, localCentroidY = 0, localCentroidZ = 0;
      double globalCentroidX = 0, globalCentroidY = 0, globalCentroidZ = 0;

      for (int i = 0; i < numPoints; i++)
      {
         int baseLocal = i * 3;
         localCentroidX += localPoints.get(baseLocal + 0);
         localCentroidY += localPoints.get(baseLocal + 1);
         localCentroidZ += localPoints.get(baseLocal + 2);

         int j = correspondences.get(i);
         int baseGlobal = j * 3;
         globalCentroidX += globalPoints.get(baseGlobal + 0);
         globalCentroidY += globalPoints.get(baseGlobal + 1);
         globalCentroidZ += globalPoints.get(baseGlobal + 2);
      }

      localCentroidX /= numPoints;
      localCentroidY /= numPoints;
      localCentroidZ /= numPoints;
      globalCentroidX /= numPoints;
      globalCentroidY /= numPoints;
      globalCentroidZ /= numPoints;

      // Step 2: Compute cross-covariance matrix H
      DMatrixRMaj H = new DMatrixRMaj(3, 3);

      for (int i = 0; i < numPoints; i++)
      {
         int baseLocal = i * 3;
         double lx = localPoints.get(baseLocal + 0) - localCentroidX;
         double ly = localPoints.get(baseLocal + 1) - localCentroidY;
         double lz = localPoints.get(baseLocal + 2) - localCentroidZ;

         int j = correspondences.get(i);
         int baseGlobal = j * 3;
         double gx = globalPoints.get(baseGlobal + 0) - globalCentroidX;
         double gy = globalPoints.get(baseGlobal + 1) - globalCentroidY;
         double gz = globalPoints.get(baseGlobal + 2) - globalCentroidZ;

         // H += local_centered * global_centered^T
         H.add(0, 0, lx * gx);
         H.add(0, 1, lx * gy);
         H.add(0, 2, lx * gz);
         H.add(1, 0, ly * gx);
         H.add(1, 1, ly * gy);
         H.add(1, 2, ly * gz);
         H.add(2, 0, lz * gx);
         H.add(2, 1, lz * gy);
         H.add(2, 2, lz * gz);
      }

      // Step 3: Compute SVD: H = U * S * V^T
      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(3, 3, true, true, false);
      svd.decompose(H);

      DMatrixRMaj U = svd.getU(null, false);
      DMatrixRMaj V = svd.getV(null, false);

      // Step 4: Compute rotation: R = V * U^T
      DMatrixRMaj Ut = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.transpose(U, Ut);

      DMatrixRMaj R = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.mult(V, Ut, R);

      // Step 5: Handle reflection case (det(R) < 0)
      if (CommonOps_DDRM.det(R) < 0)
      {
         V.set(0, 2, -V.get(0, 2));
         V.set(1, 2, -V.get(1, 2));
         V.set(2, 2, -V.get(2, 2));
         CommonOps_DDRM.mult(V, Ut, R);
      }

      // Step 6: Compute translation: t = centroid_global - R * centroid_local
      DMatrixRMaj localCentroid = new DMatrixRMaj(new double[][] {{localCentroidX}, {localCentroidY}, {localCentroidZ}});
      DMatrixRMaj globalCentroid = new DMatrixRMaj(new double[][] {{globalCentroidX}, {globalCentroidY}, {globalCentroidZ}});
      DMatrixRMaj t = new DMatrixRMaj(3, 1);

      CommonOps_DDRM.mult(R, localCentroid, t);
      CommonOps_DDRM.subtract(globalCentroid, t, t);

      // Step 7: Build 4x4 homogeneous transformation matrix
      DMatrixRMaj transform = new DMatrixRMaj(4, 4);

      // Copy rotation (top-left 3x3)
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            transform.set(i, j, R.get(i, j));
         }
      }

      // Copy translation (top-right 3x1)
      transform.set(0, 3, t.get(0, 0));
      transform.set(1, 3, t.get(1, 0));
      transform.set(2, 3, t.get(2, 0));

      // Bottom row [0, 0, 0, 1]
      transform.set(3, 0, 0);
      transform.set(3, 1, 0);
      transform.set(3, 2, 0);
      transform.set(3, 3, 1);

      return transform;
   }

}
