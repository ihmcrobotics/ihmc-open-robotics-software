package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import gnu.trove.list.array.TFloatArrayList;
import perception_msgs.HeightMapMessageForController;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;

import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

public class HeightMapCommand implements Command<HeightMapCommand, HeightMapMessageForController>
{
   private long sequenceId;

   /* List of heights indexed by key. See HeightMapTools for definition of key */
   private final TFloatArrayList heights = new TFloatArrayList();
   /* Zero-indexed key corresponding to the center of the height map */
   private int centerIndex;
   /* Number of cells along each axis of the height map */
   private int cellsPerAxis;
   /* Discretization size in meters of the height map, aka the map resolution */
   private double cellSize;
   /* Length of the height map along x/y in meters */
   private double mapSize;
   /* World-frame coordinate of the center of the height map */
   private final Point2D gridCenter = new Point2D();
   /* Convenience fields for the height map dimensions */
   private double minX, maxX, minY, maxY;

   private final Inflater inflater = new Inflater();
   private byte[] compressedCells = new byte[0];
   private byte[] cells = new byte[0];

   @Override
   public void clear()
   {
   }

   @Override
   public void setFromMessage(HeightMapMessageForController message)
   {
      this.sequenceId = message.getSequenceId();
      this.centerIndex = computeCenterIndex(message.getWidthInMeters(), message.getCellSizeInMeters());
      this.cellsPerAxis = message.getCellsPerAxis();
      this.cellSize = message.getCellSizeInMeters();
      this.mapSize = message.getWidthInMeters();
      this.gridCenter.set(message.getGridCenterX(), message.getGridCenterY());

      heights.resetQuick();
      decompressHeights(message);

      updateGridDimensions();
   }

   private void decompressHeights(HeightMapMessageForController message)
   {
      int cellCount = cellsPerAxis * cellsPerAxis;
      int compressedSize = message.getCompressedHeights().size();
      if (compressedCells.length < compressedSize)
         compressedCells = new byte[compressedSize];
      if (cells.length < 2 * cellCount)
         cells = new byte[2 * cellCount];
      for (int i = 0; i < compressedSize; i++)
         compressedCells[i] = message.getCompressedHeights().get(i);

      int size = 0;
      inflater.reset();
      inflater.setInput(compressedCells, 0, compressedSize);
      try
      {
         while (size < 2 * cellCount && !inflater.finished())
         {
            int inflated = inflater.inflate(cells, size, 2 * cellCount - size);
            if (inflated == 0 && inflater.needsInput())
               break;
            size += inflated;
         }
      }
      catch (DataFormatException e)
      {
         LogTools.error("Corrupted height map {}: {}", message.getSequenceId(), e.getMessage());
      }

      if (size != 2 * cellCount)
      {
         LogTools.error("Height map {} has {} bytes of cells, expected {}", message.getSequenceId(), size, 2 * cellCount);
         for (int i = 0; i < cellCount; i++)
            heights.add(Float.NaN);
         return;
      }

      float offset = message.getHeightOffset();
      float resolution = message.getHeightResolution();
      for (int i = 0; i < cellCount; i++)
      {
         short value = (short) ((cells[2 * i] & 0xFF) | (cells[2 * i + 1] << 8));
         heights.add(value == HeightMapMessageForController.NO_DATA_VALUE ? Float.NaN : offset + resolution * value);
      }
   }

   @Override
   public Class<HeightMapMessageForController> getMessageClass()
   {
      return HeightMapMessageForController.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(HeightMapCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.centerIndex = other.centerIndex;
      this.cellsPerAxis = other.cellsPerAxis;
      this.cellSize = other.cellSize;
      this.mapSize = other.mapSize;
      this.gridCenter.set(other.gridCenter);

      this.heights.resetQuick();
      for (int i = 0; i < other.heights.size(); i++)
      {
         this.heights.add(other.heights.get(i));
      }

      updateGridDimensions();
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeight(double x, double y)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         //LogTools.debug(String.format("Outside height map bounds: %.2f, %.2f, %.2f, %.2f", minX, maxX, minY, maxY));
         return Double.NaN;
      }

      int key = coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), cellSize, centerIndex);
      return heights.get(key);
   }

   public double getCellSize()
   {
      return cellSize;
   }

   private void updateGridDimensions()
   {
      double epsilon = 1e-8;
      double halfWidth = 0.5 * (mapSize + cellSize) - epsilon;
      minX = gridCenter.getX() - halfWidth;
      maxX = gridCenter.getX() + halfWidth;
      minY = gridCenter.getY() - halfWidth;
      maxY = gridCenter.getY() + halfWidth;
   }

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

   public static int coordinateToIndex(double coordinate, double gridCenter, double resolution, int centerIndex)
   {
      return (int) Math.round((coordinate - gridCenter) / resolution) + centerIndex;
   }

   public static double indexToCoordinate(int index, double mapCenter, double resolution, int centerIndex)
   {
      return (index - centerIndex) * resolution + mapCenter;
   }

   public static int indicesToKey(int xIndex, int yIndex, int centerIndex)
   {
      return yIndex + xIndex * (2 * centerIndex + 1);
   }
}
