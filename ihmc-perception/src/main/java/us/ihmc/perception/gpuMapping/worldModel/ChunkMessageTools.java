package us.ihmc.perception.gpuMapping.worldModel;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ChunkMessage;
import us.ihmc.euclid.tuple3D.Point3D;

public class ChunkMessageTools
{
   public static void unpackMessageToChunk(ChunkMessage chunkMessage, Chunk chunkToPack)
   {
      // Decode the PNG compressed height map data
      Mat compressedMat = new Mat(chunkMessage.getHeights().getBuffer().array());
      Mat chunkMap = new Mat(chunkMessage.getCellsPerAxis(), 2 * chunkMessage.getCellsPerAxis(), opencv_core.CV_16UC1);
      opencv_imgcodecs.imdecode(compressedMat, opencv_imgcodecs.IMREAD_UNCHANGED, chunkMap);

      // Convert the Mat into HeightMapData object
      Point3D mapOrigin = new Point3D(chunkMessage.getOriginX(), chunkMessage.getOriginY(), 0.0);

      ChunkTools.convertToChunk(chunkMap, chunkToPack, mapOrigin, chunkMessage.getWidthInMeters(), chunkMessage.getCellSizeInMeters());

      // Close pointers
      compressedMat.close();
      chunkMap.close();
   }

   public static void toMessage(Chunk chunk, ChunkMessage messageToPack)
   {
      messageToPack.setHashCodeOfChunk(chunk.hashCode());
      messageToPack.setOriginX(chunk.getOriginX());
      messageToPack.setOriginY(chunk.getOriginY());
      messageToPack.setWidthInMeters(chunk.getWidthInMeters());
      messageToPack.setCellSizeInMeters(chunk.getCellSize());
      messageToPack.setCellsPerAxis(chunk.getCellsPerAxis());

      // Get the heights in native memory as a 16U Mat
      FloatPointer dataPointer = new FloatPointer(chunk.getChunkHeights());
      Mat chunkMap16U = new Mat(chunk.getCellsPerAxis(), 2 * chunk.getCellsPerAxis(), opencv_core.CV_16U, dataPointer);

      // Compress the data using PNG
      BytePointer compressedData = new BytePointer();
      opencv_imgcodecs.imencode(".png", chunkMap16U, compressedData);

      // Pack the compressed data into the message
      int compressedDataSize = (int) compressedData.limit();
      compressedData.get(messageToPack.getHeights().getBuffer().array(), 0, compressedDataSize);
      messageToPack.getHeights().getBuffer().position(compressedDataSize);

      // Close pointers
      dataPointer.close();
      chunkMap16U.close();
      compressedData.close();
   }
}
