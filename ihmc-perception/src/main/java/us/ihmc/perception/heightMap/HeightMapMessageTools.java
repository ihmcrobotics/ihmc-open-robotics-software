package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Integer;

import java.nio.FloatBuffer;

public class HeightMapMessageTools
{
   /**
    * This method is deprecated because it's slow. Sending and receiving messages should be done with the {@link Mat} objects to increase efficiency.
    */
   @Deprecated
   public static HeightMapData unpackMessage(HeightMapMessage heightMapMessage)
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

   public static Mat unpackMessageToMat(HeightMapMessage heightMapMessage, HeightMapParameters heightMapParameters)
   {
      if (heightMapMessage == null)
         return null;

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapMessage.getGridSizeXy(), heightMapMessage.getXyResolution());
      int cellsPerAxis = 2 * centerIndex + 1;

      Mat heightMap = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);

      byte[] dataArray = new byte[cellsPerAxis * cellsPerAxis * Short.BYTES];

      for (int i = 0; i < heightMapMessage.getHeights().size(); i++)
      {
         double height = heightMapMessage.getHeights().get(i);
         int key = heightMapMessage.getKeys().get(i);

         int xIndex = key % cellsPerAxis;
         int yIndex = key / cellsPerAxis;

         int index = (yIndex * cellsPerAxis + xIndex) * Short.BYTES;

         int cellHeight = (int) ((height + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());

         // Convert short to bytes (little-endian)
         dataArray[index] = (byte) (cellHeight & 0xFF);
         dataArray[index + 1] = (byte) ((cellHeight >> 8) & 0xFF);
      }

      // Put it all at once
      heightMap.data().put(dataArray);

      return heightMap;
   }

   /**
    * This method is too slow, creating a new {@link HeightMapMessage} object is too slow when trying to
    * use this in the update loop. Especially when we start sending larger maps.
    * Please use {@link HeightMapMessageTools#toMessage(Mat, HeightMapMessage, Point3D, double, double)}
    */
   @Deprecated
   public static HeightMapMessage toMessage(HeightMapData heightMapData)
   {
      HeightMapMessage message = new HeightMapMessage();
      toMessage(heightMapData, message);

      return message;
   }

   /**
    * This methos is slow, its ok to use this if necessary, but going forward we should be using the
    * {@link HeightMapMessageTools#toMessage(Mat, HeightMapMessage, Point3D, double, double)} as its faster
    */
   @Deprecated
   public static void toMessage(HeightMapData heightMapData, HeightMapMessage messageToPack)
   {
      clear(messageToPack);

      messageToPack.setGridSizeXy(heightMapData.getGridSizeXY());
      messageToPack.setXyResolution(heightMapData.getGridResolutionXY());
      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      messageToPack.setEstimatedGroundHeight(heightMapData.getEstimatedGroundHeight());

      for (int i = 0; i < heightMapData.getNumberOfOccupiedCells() && i < messageToPack.getKeys().capacity(); i++)
      {
         int key = heightMapData.getKey(i);
         messageToPack.getKeys().add(key);
         messageToPack.getHeights().add((float) heightMapData.getHeightAt(key));
      }
   }

   public static void toMessage(Mat heightMapDataForMessage,
                                HeightMapMessage messageToPack,
                                Point3D heightMapCenter,
                                double widthInMeters,
                                double cellSizeInMeters)
   {
      clear(messageToPack);

      messageToPack.setGridSizeXy(widthInMeters);
      messageToPack.setXyResolution(cellSizeInMeters);
      messageToPack.setGridCenterX(heightMapCenter.getX());
      messageToPack.setGridCenterY(heightMapCenter.getY());

      // Guarantee the width is at meter increments. So we can't have 4.02, that becomes 4.0
      widthInMeters = (float) (Math.floor(widthInMeters / cellSizeInMeters) * cellSizeInMeters);
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Make sure Mat type is correct
      if (heightMapDataForMessage.type() != opencv_core.CV_32FC1)
         throw new IllegalArgumentException("Expected CV_32FC1 Mat");

      FloatBuffer floatBuffer = heightMapDataForMessage.createBuffer(); // or ByteBuffer -> FloatBuffer

      // This is done for speed optimization
      float[] heightsArray = new float[totalCells];
      floatBuffer.get(heightsArray);
      Integer keys = messageToPack.getKeys();
      Float heights = messageToPack.getHeights();

      // No overhead for this loop, it's as fast as possible with the current message
      for (int i = 0; i < totalCells; ++i)
      {
         keys.add(i);
         heights.add(heightsArray[i]);
      }
   }

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
            message.getHeights().add(0.0f);
         }
      }
   }
}