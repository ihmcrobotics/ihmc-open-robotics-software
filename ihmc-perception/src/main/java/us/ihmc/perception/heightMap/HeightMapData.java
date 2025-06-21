package us.ihmc.perception.heightMap;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.Arrays;

public class HeightMapData
{
   /* Unordered list of the keys of all occupied cells */
   private final TIntArrayList occupiedCells = new TIntArrayList();
   /* List of heights indexed by key */
   private Vector3D[] normals;

   private FloatIndexer heightsIndexer;
   private Mat heights;

   private int centerIndex;
   private int cellsPerAxis;
   private double gridResolutionXY;
   private double gridSizeXY;
   private final Point2D gridCenter = new Point2D();
   private double estimatedGroundHeight = Double.NaN;

   private double minX, maxX, minY, maxY;

   public HeightMapData(double gridResolutionXY, double gridSizeXY, double gridCenterX, double gridCenterY)
   {
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = 2 * centerIndex + 1;

      heights = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1); // OpenCV uses row-major
      heightsIndexer = heights.createIndexer();
      for (int y = 0; y < cellsPerAxis; y++)
      {
         for (int x = 0; x < cellsPerAxis; x++)
         {
            heightsIndexer.put(x, y, Float.NaN);
         }
      }

      this.normals = new Vector3D[cellsPerAxis * cellsPerAxis];
      this.gridCenter.set(gridCenterX, gridCenterY);

      double epsilon = 1e-8;
      double halfWidth = 0.5 * (gridSizeXY + gridResolutionXY) - epsilon;
      minX = gridCenterX - halfWidth;
      maxX = gridCenterX + halfWidth;
      minY = gridCenterY - halfWidth;
      maxY = gridCenterY + halfWidth;

      reset();
   }

   public HeightMapData(HeightMapData latestHeightMapData)
   {
      set(latestHeightMapData);
   }

   public void set(HeightMapData latestHeightMapData)
   {
      this.gridResolutionXY = latestHeightMapData.getGridResolutionXY();
      this.gridSizeXY = latestHeightMapData.getGridSizeXY();
      this.centerIndex = HeightMapTools.computeCenterIndex(latestHeightMapData.getGridSizeXY(), latestHeightMapData.getGridResolutionXY());
      this.cellsPerAxis = 2 * latestHeightMapData.getCenterIndex() + 1;
      this.estimatedGroundHeight = latestHeightMapData.getEstimatedGroundHeight();

      this.heights = new Mat(latestHeightMapData.getCellsPerAxis(), latestHeightMapData.getCellsPerAxis(), opencv_core.CV_32FC1);
      heightsIndexer = heights.createIndexer();

      this.normals = new Vector3D[latestHeightMapData.getCellsPerAxis() * latestHeightMapData.getCellsPerAxis()];

      for (int y = 0; y < latestHeightMapData.getCellsPerAxis(); y++)
      {
         for (int x = 0; x < latestHeightMapData.getCellsPerAxis(); x++)
         {
            float height = latestHeightMapData.getHeightsIndexer().get(y, x);
            heightsIndexer.put(x, y, height);

            int key = y * cellsPerAxis + x;
            Vector3D normal = latestHeightMapData.normals[key];
            normals[key] = (normal != null) ? new Vector3D(normal) : null;
         }
      }

      this.gridCenter.set(latestHeightMapData.getGridCenter());

      occupiedCells.addAll(latestHeightMapData.occupiedCells);

      double epsilon = 1e-8;
      double halfWidth = 0.5 * (this.gridSizeXY + this.gridResolutionXY) - epsilon;
      minX = this.gridCenter.getX() - halfWidth;
      maxX = this.gridCenter.getX() + halfWidth;
      minY = this.gridCenter.getY() - halfWidth;
      maxY = this.gridCenter.getY() + halfWidth;
   }

   public void reset()
   {
      occupiedCells.clear();
      for (int y = 0; y < cellsPerAxis; y++)
      {
         for (int x = 0; x < cellsPerAxis; x++)
         {
            heightsIndexer.put(x, y, Float.NaN);
         }
      }

      Arrays.fill(normals, null);
      estimatedGroundHeight = Double.NaN;
   }

   public boolean isEmpty()
   {
      return occupiedCells.isEmpty();
   }

   public double getGridResolutionXY()
   {
      return gridResolutionXY;
   }

   public double getGridSizeXY()
   {
      return gridSizeXY;
   }

   public FloatIndexer getHeightsIndexer()
   {
      return heightsIndexer;
   }

   public Mat getHeightMat()
   {
      return heights;
   }

   public int getNumberOfOccupiedCells()
   {
      return occupiedCells.size();
   }

   public double getHeight(int i)
   {
      int key = occupiedCells.get(i);
      int x = HeightMapTools.keyToXIndex(key, centerIndex);
      int y = HeightMapTools.keyToYIndex(key, centerIndex);
      return heightsIndexer.get(x, y);
   }

   public Point2D getCellPosition(int i)
   {
      int key = occupiedCells.get(i);
      return new Point2D(HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), gridResolutionXY, centerIndex),
                         HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), gridResolutionXY, centerIndex));
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeightAt(double x, double y)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         return Double.NaN;
      }

      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenter.getX(), gridResolutionXY, centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenter.getY(), gridResolutionXY, centerIndex);

      float height = heightsIndexer.get(xIndex, yIndex);
      return Float.isNaN(height) ? estimatedGroundHeight : height;
   }

   public void setHeightAt(int key, double height)
   {
      setHeightAt(key, height, null);
   }

   public void setHeightAt(int key, double height, Vector3DReadOnly normal)
   {
      int xIndex = HeightMapTools.keyToXIndex(key, centerIndex);
      int yIndex = HeightMapTools.keyToYIndex(key, centerIndex);

      float currentHeight = heightsIndexer.get(xIndex, yIndex);
      if (Float.isNaN(currentHeight))
      {
         occupiedCells.add(key);
      }

      heightsIndexer.put(xIndex, yIndex, (float) height);

      if (normal != null)
      {
         normals[key] = new Vector3D(normal);
      }
   }

   public void setHeightAt(double x, double y, double z)
   {
      setHeightAt(x, y, z, null);
   }

   public void setHeightAt(double x, double y, double z, Vector3DReadOnly normal)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         return;
      }

      int xIndex = HeightMapTools.coordinateToIndex(x, gridCenter.getX(), gridResolutionXY, centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(y, gridCenter.getY(), gridResolutionXY, centerIndex);
      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), gridResolutionXY, centerIndex);

      float currentHeight = heightsIndexer.get(xIndex, yIndex);
      if (Float.isNaN(currentHeight))
      {
         occupiedCells.add(key);
      }

      heightsIndexer.put(xIndex, yIndex, (float) z);

      if (normal != null)
      {
         normals[key] = new Vector3D(normal);
      }
   }

   public double getHeightAt(int key)
   {
      int x = HeightMapTools.keyToXIndex(key, centerIndex);
      int y = HeightMapTools.keyToYIndex(key, centerIndex);
      float height = heightsIndexer.get(x, y);
      return Float.isNaN(height) ? estimatedGroundHeight : height;
   }

   public double getHeightAt(int xIndex, int yIndex)
   {
      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         return Double.NaN;
      }

      float height = heightsIndexer.get(xIndex, yIndex);
      return Float.isNaN(height) ? estimatedGroundHeight : height;
   }

   public boolean isCellAtGroundPlane(int xIndex, int yIndex)
   {
      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
         return true; // or false, depending on how you want to treat out-of-bounds

      float height = heightsIndexer.get(xIndex, yIndex);
      return Float.isNaN(height);
   }

   public void setEstimatedGroundHeight(double estimatedGroundHeight)
   {
      this.estimatedGroundHeight = estimatedGroundHeight;
   }

   public double getEstimatedGroundHeight()
   {
      return estimatedGroundHeight;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public Point2D getGridCenter()
   {
      return gridCenter;
   }

   public double getMinHeight()
   {
      float minValue = Float.POSITIVE_INFINITY;

      for (int y = 0; y < cellsPerAxis; y++)
      {
         for (int x = 0; x < cellsPerAxis; x++)
         {
            float height = heightsIndexer.get(x, y);
            if (!Float.isNaN(height) && height < minValue)
            {
               minValue = height;
            }
         }
      }

      return minValue == Float.POSITIVE_INFINITY ? Double.NaN : minValue;
   }

   public int getKey(int i)
   {
      return occupiedCells.get(i);
   }

   public void markGroundCell(int i)
   {
      int key = occupiedCells.get(i);
      int x = HeightMapTools.keyToXIndex(key, centerIndex);
      int y = HeightMapTools.keyToYIndex(key, centerIndex);

      heightsIndexer.put(x, y, Float.NaN);
      normals[key] = null;
      occupiedCells.remove(i);
   }

   public void setGridCenter(double x, double y)
   {
      gridCenter.set(x, y);

      double epsilon = 1e-8;
      double halfWidth = 0.5 * (gridSizeXY + gridResolutionXY) - epsilon;
      minX = gridCenter.getX() - halfWidth;
      maxX = gridCenter.getX() + halfWidth;
      minY = gridCenter.getY() - halfWidth;
      maxY = gridCenter.getY() + halfWidth;
   }

   public double[] getHeights()
   {
      double[] data = new double[cellsPerAxis * cellsPerAxis];
      for (int y = 0; y < cellsPerAxis; y++)
      {
         for (int x = 0; x < cellsPerAxis; x++)
         {
            data[y * cellsPerAxis + x] = heightsIndexer.get(x, y);
         }
      }
      return data;
   }
}
