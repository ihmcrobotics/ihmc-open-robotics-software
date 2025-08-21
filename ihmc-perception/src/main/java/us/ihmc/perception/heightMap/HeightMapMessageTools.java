package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Integer;

import java.nio.FloatBuffer;
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

      HeightMapData heightMapData = new HeightMapData(heightMapMessage.getCellSizeInMeters(),
                                                      heightMapMessage.getWidthInMeters(),
                                                      heightMapMessage.getGridCenterX(),
                                                      heightMapMessage.getGridCenterY());

      for (int key = 0; key < heightMapMessage.getHeights().size(); key++)
      {
         double height = heightMapMessage.getHeights().get(key);
         heightMapData.setHeight(key, height);
      }

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

      for (int key = 0; key < chunkMessage.getHeights().size(); key++)
      {
         heights[key] = (short) chunkMessage.getHeights().get(key);
      }

      shortBuffer.put(heights);

      return heightMap;
   }

   public static Mat unpackMessageToMat(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null)
         return null;

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getWidthInMeters(), heightMapMessage.getCellSizeInMeters());
      int cellsPerAxis = 2 * centerIndex + 1;

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      FloatBuffer floatBuffer = heightMap.createBuffer();

      int totalCells = cellsPerAxis * cellsPerAxis;
      float[] heights = new float[totalCells];

      // Optimization of caching the arrays
      Float heightsFromMessage = heightMapMessage.getHeights();

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         heights[i] = heightsFromMessage.get(i);
      }

      floatBuffer.put(heights);

      return heightMap;
   }

   /**
    * This method is too slow, creating a new {@link HeightMapMessage} object is too slow when trying to
    * use this in the update loop. Especially when we start sending larger maps.
    * Please use {@link #toMessage(Mat, float[], HeightMapMessage, Point3D, double, double, int)}
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
    * {@link #toMessage(Mat, float[], HeightMapMessage, Point3D, double, double, int)} as its faster
    */
   @Deprecated
   public static void toMessage(HeightMapData heightMapData, HeightMapMessage messageToPack)
   {
      clear(messageToPack);

      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      int numberOfCells = heightMapData.getCellsPerAxis() * heightMapData.getCellsPerAxis();

      for (int key = 0; key < numberOfCells; key++)
      {
         messageToPack.getHeights().add((short) heightMapData.getHeight(key));
      }
   }

   public static void toMessage(Mat chunkDataForMessage,
                                ChunkMessage messageToPack,
                                Point3D mapOrigin,
                                double widthInMeters,
                                double cellSizeInMeters,
                                int cellsPerAxis)
   {
      messageToPack.setOriginX(mapOrigin.getX());
      messageToPack.setOriginY(mapOrigin.getY());
      messageToPack.setWidthInMeters(widthInMeters);
      messageToPack.setCellSizeInMeters(cellSizeInMeters);
      messageToPack.setCellsPerAxis(cellsPerAxis);

      // Guarantee the width is at meter increments. So we can't have 4.02, that becomes 4.0
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Make sure Mat type is correct
      if (chunkDataForMessage.type() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Expected CV_16UC1 Mat");

      FloatBuffer floatBuffer = chunkDataForMessage.createBuffer(); // or ByteBuffer -> ShortBuffer

      // This is done for speed optimization
      float[] heightsArray = new float[totalCells];
      floatBuffer.get(heightsArray);
      Float heights = messageToPack.getHeights();

      // No overhead for this loop, it's as fast as possible (according to AI) with the current message
      for (int i = 0; i < totalCells; ++i)
      {
         if (i < heights.size())
            heights.set(i, heightsArray[i]);
         else
            heights.add(heightsArray[i]);
      }
   }

   public static void clear(ChunkMessage messageToClear)
   {
      messageToClear.setOriginX(-1.0);
      messageToClear.setOriginX(-1.0);
      messageToClear.getHeights().clear();
   }

   /**
    * This method is meant to be as fast as possible, which is why we are passing in the {@link  HeightMapMessage} and the heights array to this method.
    * Allocating that memory in the update loop slows things down so we avoid that.
    */
   public static void toMessage(Mat heightMapDataForMessage,
                                float[] heightsArray,
                                HeightMapMessage messageToPack,
                                Point3D heightMapCenter,
                                double widthInMeters,
                                double cellSizeInMeters,
                                int cellsPerAxis)
   {
      messageToPack.setGridCenterX(heightMapCenter.getX());
      messageToPack.setGridCenterY(heightMapCenter.getY());
      messageToPack.setWidthInMeters(widthInMeters);
      messageToPack.setCellSizeInMeters(cellSizeInMeters);
      messageToPack.setCellsPerAxis(cellsPerAxis);

      // Guarantee the width is at meter increments. So we can't have 4.02, that becomes 4.0
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Make sure Mat type is correct
      if (heightMapDataForMessage.type() != opencv_core.CV_32FC1)
         throw new IllegalArgumentException("Expected CV_32FC1 Mat");

      FloatBuffer floatBuffer = heightMapDataForMessage.createBuffer(); // or ByteBuffer -> ShortBuffer

      // This is done for speed optimization
      floatBuffer.get(heightsArray);
      Float heights = messageToPack.getHeights();

      // No overhead for this loop, it's as fast as possible (according to AI) with the current message
      for (int i = 0; i < totalCells; ++i)
      {
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
      messageToClear.setGridCenterX(-1.0);
      messageToClear.setGridCenterY(-1.0);

      messageToClear.getHeights().clear();
   }

   public static void setToFlatGround(HeightMapMessage message)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(message.getWidthInMeters(), message.getCellSizeInMeters());
      int cellsPerAxis = 2 * centerIndex + 1;

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            message.getHeights().add(0);
         }
      }
   }
}