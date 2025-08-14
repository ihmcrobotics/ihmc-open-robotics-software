package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Integer;

import java.nio.ShortBuffer;

public class HeightMapMessageTools
{
   /**
    * This method is deprecated because it's slow. Sending and receiving messages should be done with the {@link Mat} objects to increase efficiency.
    */
   @Deprecated
   public static HeightMapData unpackMessageToHeightMapData(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null)
         return null;

      HeightMapData heightMapData = new HeightMapData(heightMapMessage.getXyResolution(),
                                                      heightMapMessage.getGridSizeXy(),
                                                      heightMapMessage.getGridCenterX(),
                                                      heightMapMessage.getGridCenterY());

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         double height = heightMapMessage.getHeights().get(i);
         int key = heightMapMessage.getKeys().get(i);
         heightMapData.setHeightAt(key, height);
      }

      heightMapData.setEstimatedGroundHeight(heightMapMessage.getEstimatedGroundHeight());
      return heightMapData;
   }

   public static Mat unpackMessageToMat(ChunkMessage chunkMessage)
   {
      if (chunkMessage == null)
         return null;

      int cellsPerAxis = chunkMessage.getCellsPerAxis();

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      ShortBuffer shortBuffer = heightMap.createBuffer();

      int totalCells = cellsPerAxis * cellsPerAxis;
      short[] heights = new short[totalCells];

      for (int i = 0; i < chunkMessage.getHeights().size(); i++)
      {
         short height = (short) chunkMessage.getHeights().get(i);
         int key = chunkMessage.getKeys().get(i);

         int xIndex = key % cellsPerAxis;
         int yIndex = key / cellsPerAxis;

         int index = yIndex * cellsPerAxis + xIndex;
         heights[index] = height;
      }

      shortBuffer.put(heights);

      return heightMap;
   }

   public static Mat unpackMessageToMat(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null)
         return null;

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getGridSizeXy(), heightMapMessage.getXyResolution());
      int cellsPerAxis = 2 * centerIndex + 1;

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      ShortBuffer shortBuffer = heightMap.createBuffer();

      int totalCells = cellsPerAxis * cellsPerAxis;
      short[] heights = new short[totalCells];

      // Optimization of caching the arrays
      Integer heightsFromMessage = heightMapMessage.getHeights();
      Integer keysFromMessage = heightMapMessage.getKeys();

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         int key = keysFromMessage.get(i);
         heights[key] = (short) heightsFromMessage.get(i);
      }

      shortBuffer.put(heights);

      return heightMap;
   }

   /**
    * This method is too slow, creating a new {@link HeightMapMessage} object is too slow when trying to
    * use this in the update loop. Especially when we start sending larger maps.
    * Please use {@link HeightMapMessageTools#toMessage(Mat, HeightMapMessage, Point3D, double, double, double, double, int)}
    * I'm going to say it one more time in case it wasn't clear the first time. THIS METHOD IS TOO SLOW, DO NOT USE!
    */
   @Deprecated
   public static HeightMapMessage toMessage(HeightMapData heightMapData)
   {
      HeightMapMessage message = new HeightMapMessage();
      toMessage(heightMapData, message);

      return message;
   }

   /**
    * This method is slow, it's ok to use this if necessary, but going forward we should be using the
    * {@link HeightMapMessageTools#toMessage(Mat, HeightMapMessage, Point3D, double, double, double, double, int)} as its faster
    */
   @Deprecated
   public static void toMessage(HeightMapData heightMapData, HeightMapMessage messageToPack)
   {
      clear(messageToPack);

      messageToPack.setGridSizeXy(heightMapData.getMapSize());
      messageToPack.setXyResolution(heightMapData.getCellSize());
      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      messageToPack.setEstimatedGroundHeight(heightMapData.getEstimatedGroundHeight());

      for (int i = 0; i < heightMapData.getNumberOfOccupiedCells() && i < messageToPack.getKeys().capacity(); i++)
      {
         int key = heightMapData.getKey(i);
         messageToPack.getKeys().add((key));
         messageToPack.getHeights().add((short) heightMapData.getHeightAt(key));
      }
   }

   public static void toMessage(Mat chunkDataForMessage,
                                ChunkMessage messageToPack,
                                Point3D mapOrigin,
                                double widthInMeters,
                                double cellSizeInMeters,
                                double heightOffset,
                                double heightScaleFactor,
                                int cellsPerAxis)
   {
      messageToPack.setGridSizeXy(widthInMeters);
      messageToPack.setXyResolution(cellSizeInMeters);
      messageToPack.setOriginX(mapOrigin.getX());
      messageToPack.setOriginY(mapOrigin.getY());
      messageToPack.setWidthInMeters(widthInMeters);
      messageToPack.setCellSizeInMeters(cellSizeInMeters);
      messageToPack.setHeightOffset(heightOffset);
      messageToPack.setHeightScaleFactor(heightScaleFactor);
      messageToPack.setCellsPerAxis(cellsPerAxis);

      // Guarantee the width is at meter increments. So we can't have 4.02, that becomes 4.0
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Make sure Mat type is correct
      if (chunkDataForMessage.type() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Expected CV_16UC1 Mat");

      ShortBuffer shortBuffer = chunkDataForMessage.createBuffer(); // or ByteBuffer -> ShortBuffer

      // This is done for speed optimization
      short[] heightsArray = new short[totalCells];
      shortBuffer.get(heightsArray);
      Integer keys = messageToPack.getKeys();
      Integer heights = messageToPack.getHeights();

      // No overhead for this loop, it's as fast as possible (according to AI) with the current message
      for (int i = 0; i < totalCells; ++i)
      {
         if (i < keys.size())
            keys.set(i, i);
         else
            keys.add(i);

         if (i < heights.size())
            heights.set(i, heightsArray[i]);
         else
            heights.add(heightsArray[i]);
      }
   }

   public static void clear(ChunkMessage messageToClear)
   {
      messageToClear.setGridSizeXy(-1.0);
      messageToClear.setXyResolution(-1.0);
      messageToClear.setOriginX(-1.0);
      messageToClear.setOriginX(-1.0);
      messageToClear.setEstimatedGroundHeight(-1.0);
      messageToClear.getKeys().clear();
      messageToClear.getHeights().clear();
   }

   public static void toMessage(Mat heightMapDataForMessage,
                                HeightMapMessage messageToPack,
                                Point3D heightMapCenter,
                                double widthInMeters,
                                double cellSizeInMeters,
                                double heightOffset,
                                double heightScaleFactor,
                                int cellsPerAxis)
   {
      messageToPack.setGridSizeXy(widthInMeters);
      messageToPack.setXyResolution(cellSizeInMeters);
      messageToPack.setGridCenterX(heightMapCenter.getX());
      messageToPack.setGridCenterY(heightMapCenter.getY());
      messageToPack.setWidthInMeters(widthInMeters);
      messageToPack.setCellSizeInMeters(cellSizeInMeters);
      messageToPack.setHeightOffset(heightOffset);
      messageToPack.setHeightScaleFactor(heightScaleFactor);
      messageToPack.setCellsPerAxis(cellsPerAxis);

      // Guarantee the width is at meter increments. So we can't have 4.02, that becomes 4.0
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Make sure Mat type is correct
      if (heightMapDataForMessage.type() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Expected CV_16UC1 Mat");

      ShortBuffer shortBuffer = heightMapDataForMessage.createBuffer(); // or ByteBuffer -> ShortBuffer

      // This is done for speed optimization
      short[] heightsArray = new short[totalCells];
      shortBuffer.get(heightsArray);
      Integer keys = messageToPack.getKeys();
      Integer heights = messageToPack.getHeights();

      // No overhead for this loop, it's as fast as possible (according to AI) with the current message
      for (int i = 0; i < totalCells; ++i)
      {
         if (i < keys.size())
            keys.set(i, i);
         else
            keys.add(i);

         if (i < heights.size())
            heights.set(i, heightsArray[i]);
         else
            heights.add(heightsArray[i]);
      }
   }

   /**
    * We don't want to do this unless we have too, it's too slow
    */
   @Deprecated
   public static void clear(HeightMapMessage messageToClear)
   {
      messageToClear.setGridSizeXy(-1.0);
      messageToClear.setXyResolution(-1.0);
      messageToClear.setGridCenterX(-1.0);
      messageToClear.setGridCenterY(-1.0);
      messageToClear.setEstimatedGroundHeight(-1.0);

      messageToClear.getKeys().clear();
      messageToClear.getHeights().clear();
      messageToClear.getNormals().clear();
      messageToClear.getVariances().clear();
      messageToClear.getCentroids().clear();
   }

   public static void setToFlatGround(HeightMapMessage message)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(message.getGridSizeXy(), message.getXyResolution());
      int cellsPerAxis = 2 * centerIndex + 1;

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            message.getKeys().add(key);
            message.getHeights().add(0);
         }
      }
   }
}