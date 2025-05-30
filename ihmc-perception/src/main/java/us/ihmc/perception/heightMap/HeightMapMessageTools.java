package us.ihmc.perception.heightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.euclid.tuple3D.Point3D;

public class HeightMapMessageTools
{
   public static void convertToHeightMapData(Mat heightMapPointer,
                                             HeightMapData heightMapDataToPack,
                                             Point3D gridCenter,
                                             float widthInMeters,
                                             float cellSizeInMeters,
                                             HeightMapParameters heightMapParameters)
   {
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      heightMapDataToPack.setGridCenter(gridCenter.getX(), gridCenter.getY());

      // Read data into byte[]
      byte[] data = new byte[Short.BYTES * totalCells];
      heightMapPointer.data().get(data);

      // Put height values into HeightMapData object
      for (int i = 0; i < totalCells; ++i)
      {
         // Get the start index of the bytes for a short
         int dataIndex = Short.BYTES * i;

         // Get the most and least significant bits, combine into integer
         int major = (data[dataIndex + 1] << 8) & 0xFF00;
         int minor = data[dataIndex] & 0x00FF;
         int height = major | minor;

         // Calculate cell height
         float cellHeight = (float) (((float) height / heightMapParameters.getHeightScaleFactor()) - heightMapParameters.getHeightOffset());

         // Put it into the HeightMapData object
         int key = cellsPerAxis * (i % cellsPerAxis) + (i / cellsPerAxis);
         heightMapDataToPack.setHeightAt(key, cellHeight);
      }
   }

   public static Mat convertHeightMapDataToMat(HeightMapData heightMapData, HeightMapParameters heightMapParameters)
   {
      int cellsPerAxis = heightMapData.getCellsPerAxis();
      int centerIndex = heightMapData.getCenterIndex();

      // Create a new Mat object to hold the height map data
      Mat heightMapMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            double cellHeight = heightMapData.getHeightAt(key);

            // Reverse the height calculation to get the raw height value
            int height = (int) ((cellHeight + (float) heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());

            // Store the height value in the Mat object
            heightMapMat.ptr(xIndex, yIndex).putShort((short) height);
         }
      }

      return heightMapMat;
   }

   public static void convertToHeightMapImage(ImageMessage imageMessage, Mat heightMapImageToPack)
   {
      int numberOfBytes = imageMessage.getData().size();

      // Create a pointer to the compressed data
      BytePointer compressedDataPointer = new BytePointer(imageMessage.getData().getBuffer().position(0));
      compressedDataPointer.limit(numberOfBytes);

      // Wrap the pointer in a Mat (for the imdecode function)
      Mat compressedDataMat = new Mat(1, numberOfBytes, opencv_core.CV_8UC1, compressedDataPointer);

      // Decompress the height map image
      opencv_imgcodecs.imdecode(compressedDataMat, opencv_imgcodecs.IMREAD_UNCHANGED, heightMapImageToPack);

      // Close pointers
      compressedDataMat.close();
      compressedDataPointer.close();
   }

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

   public static HeightMapMessage toMessage(HeightMapData heightMapData)
   {
      HeightMapMessage message = new HeightMapMessage();
      toMessage(heightMapData, message);

      return message;
   }

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
