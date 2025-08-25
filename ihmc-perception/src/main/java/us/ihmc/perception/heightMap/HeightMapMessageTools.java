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

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      FloatBuffer floatBuffer = heightMap.createBuffer();

      int totalCells = cellsPerAxis * cellsPerAxis;
      float[] heights = new float[totalCells];

      for (int key = 0; key < chunkMessage.getHeights().size(); key++)
      {
         heights[key] = chunkMessage.getHeights().get(key);
      }

      floatBuffer.put(heights);

      return heightMap;
   }

   public static void toMessage(HeightMapData heightMapData, HeightMapMessage messageToPack)
   {
      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      messageToPack.setWidthInMeters(heightMapData.getMapSize());
      messageToPack.setCellSizeInMeters(heightMapData.getCellSize());
      messageToPack.setCellsPerAxis(heightMapData.getCellsPerAxis());
      int totalCells = heightMapData.getCellsPerAxis() *  heightMapData.getCellsPerAxis();

      // This is done for speed optimization
      double[] heightsFromData = heightMapData.getHeights();
      Float heights = messageToPack.getHeights();

      for (int i = 0; i < totalCells; i++)
      {
         if (i < heights.size())
            heights.set(i, (float) heightsFromData[i]);
         else
            heights.add((float) heightsFromData[i]);
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
      if (chunkDataForMessage.type() != opencv_core.CV_32FC1)
         throw new IllegalArgumentException("Expected CV_32FC1 Mat");

      FloatBuffer floatBuffer = chunkDataForMessage.createBuffer();

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
    * We don't want to do this unless we have too, it's too slow
    */
   @Deprecated
   public static void clear(HeightMapMessage messageToClear)
   {
      messageToClear.setGridCenterX(-1.0);
      messageToClear.setGridCenterY(-1.0);

      messageToClear.getHeights().clear();
   }
}