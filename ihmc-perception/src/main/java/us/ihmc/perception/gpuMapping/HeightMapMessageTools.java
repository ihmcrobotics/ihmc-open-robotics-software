package us.ihmc.perception.gpuMapping;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.HeightMapMessage;
import perception_msgs.HeightMapMessageForController;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;

public class HeightMapMessageTools
{
   /**
    * There is no safety for this method, in the sense that we've got an array of floats coming from the message, and we've got data to put the floats into.
    * Its expected that the data matches the size requirements of the message.
    * By not having a safety check (i < size()) we can drastically speed up the computational time of this method.
    *
    * @param heightMapMessage the message that contains the data to put into the {@link HeightMapData}
    * @return the {@link HeightMapData} that is filled with the floats from the message
    */
   public static HeightMapData unpackMessageToHeightMapData(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null)
         return null;

      HeightMapData heightMapData = new HeightMapData(heightMapMessage.getCellSizeInMeters(),
                                                      heightMapMessage.getWidthInMeters(),
                                                      heightMapMessage.getGridCenterX(),
                                                      heightMapMessage.getGridCenterY());

      // Decode the PNG compressed height map data
      Mat compressedMat = new Mat(heightMapMessage.getHeights().getBuffer().array());
      Mat heightMapMat = new Mat(heightMapMessage.getCellsPerAxis(), 2 * heightMapMessage.getCellsPerAxis(), opencv_core.CV_16UC1);
      opencv_imgcodecs.imdecode(compressedMat, opencv_imgcodecs.IMREAD_UNCHANGED, heightMapMat);

      // Convert the Mat into HeightMapData object
      Point3D mapOrigin = new Point3D(heightMapMessage.getGridCenterX(), heightMapMessage.getGridCenterY(), 0.0);
      HeightMapTools.convertToHeightMapData(heightMapMat,
                                            heightMapData,
                                            mapOrigin,
                                            (float) heightMapMessage.getWidthInMeters(),
                                            (float) heightMapMessage.getCellSizeInMeters());

      // Close pointers
      compressedMat.close();
      heightMapMat.close();

      return heightMapData;
   }

   /**
    * There is no safety for this method, in the sense that we've got an array of floats, and we've got a message to pack with the data.
    * Its expected that the data matches the size requirements of the message.
    * By not having a safety check (i < size()) we can drastically speed up the computational time of this method.
    *
    * @param heightMapData is the data that contains the heights to be put into the message
    * @param messageToPack is the message that we want to fill up with our data to publish over the network.
    */
   public static void toMessage(HeightMapData heightMapData, HeightMapMessage messageToPack)
   {
      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      messageToPack.setWidthInMeters(heightMapData.getMapSize());
      messageToPack.setCellSizeInMeters(heightMapData.getCellSize());
      messageToPack.setCellsPerAxis(heightMapData.getCellsPerAxis());

      // Get the heights in native memory as a 16U Mat
      FloatPointer dataPointer = new FloatPointer(heightMapData.getHeights());
      Mat heightMapMat16U = new Mat(heightMapData.getCellsPerAxis(), 2 * heightMapData.getCellsPerAxis(), opencv_core.CV_16U, dataPointer);

      // Compress the data using PNG
      BytePointer compressedData = new BytePointer();
      opencv_imgcodecs.imencode(".png", heightMapMat16U, compressedData);

      // Pack the compressed data into the message
      int compressedDataSize = (int) compressedData.limit();
      compressedData.get(messageToPack.getHeights().getBuffer().array(), 0, compressedDataSize);
      messageToPack.getHeights().getBuffer().position(compressedDataSize);

      // Close pointers
      dataPointer.close();
      heightMapMat16U.close();
      compressedData.close();
   }

   /**
    * There is no safety for this method, in the sense that we've got an array of floats, and we've got a message to pack with the data.
    * Its expected that the data matches the size requirements of the message.
    * By not having a safety check (i < size()) we can drastically speed up the computational time of this method.
    *
    * @param heightMapData is the data that contains the heights to be put into the message
    * @param messageToPack is the message that we want to fill up with our data to publish over the network.
    */
   public static void toMessageForController(HeightMapData heightMapData, HeightMapMessageForController messageToPack)
   {
      messageToPack.setGridCenterX(heightMapData.getGridCenter().getX());
      messageToPack.setGridCenterY(heightMapData.getGridCenter().getY());
      messageToPack.setWidthInMeters(heightMapData.getMapSize());
      messageToPack.setCellSizeInMeters(heightMapData.getCellSize());
      messageToPack.setCellsPerAxis(heightMapData.getCellsPerAxis());

      float[] heightsFromData = heightMapData.getHeights();

      messageToPack.getHeights().getBuffer().reset();
      IDLFloatSequence heights = messageToPack.getHeights();
      heights.addAll(heightsFromData);
   }
   /**
    * We don't want to do this unless we have too, it's too slow
    */
   @Deprecated
   public static void clear(HeightMapMessage messageToClear)
   {
      messageToClear.setGridCenterX(-1.0);
      messageToClear.setGridCenterY(-1.0);

      messageToClear.getHeights().getBuffer().reset();
   }
}