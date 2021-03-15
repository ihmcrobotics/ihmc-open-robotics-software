package us.ihmc.communication.packets;

import controller_msgs.msg.dds.LidarScanMessage;
import gnu.trove.list.array.TByteArrayList;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.tools.compression.LZ4CompressionImplementation;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

/**
 * This class should be used to create and unpack a {@link LidarScanMessage}.
 */
public class LidarPointCloudCompression
{
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = ThreadLocal.withInitial(LZ4CompressionImplementation::new);

   public static void compressPointCloud(int pointCloudSize, LidarScanMessage messageToPack, LidarPointCoordinateFunction coordinateFunction)
   {
      // 3 coordinates per point, 4 bytes per coordinate
      int pointCloudSizeInBytes = 4 * 3 * pointCloudSize;

      ByteBuffer byteBuffer = ByteBuffer.allocate(pointCloudSizeInBytes);
      FloatBuffer pointCloudBuffer = byteBuffer.asFloatBuffer();

      for (int i = 0; i < pointCloudSize; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            pointCloudBuffer.put(coordinateFunction.getCoordinate(i, j));
         }
      }

      ByteBuffer compressedPointCloudBuffer = ByteBuffer.allocate(pointCloudSizeInBytes);
      LZ4CompressionImplementation compressor = compressorThreadLocal.get();
      int compressedPointCloudSize;
      try
      {
         compressedPointCloudSize = compressor.compress(byteBuffer, compressedPointCloudBuffer);
      }
      catch (LZ4Exception e)
      {
         // TODO See why the compression would fail and how to fix it.
         e.printStackTrace();
         return;
      }

      compressedPointCloudBuffer.flip();
      for (int i = 0; i < compressedPointCloudSize; i++)
      {
         messageToPack.getScan().add(compressedPointCloudBuffer.get());
      }

      messageToPack.setNumberOfPoints(pointCloudSize);
   }

   public static void decompressPointCloud(TByteArrayList compressedPointCloud,
                                           int numberOfPoints,
                                           LidarPointConsumer pointCoordinateConsumer)
   {
      ByteBuffer compressedPointCloudByteBuffer = ByteBuffer.wrap(compressedPointCloud.toArray());
      int numberOfDecompressedBytes = numberOfPoints * 4 * 3;
      ByteBuffer decompressedPointCloudByteBuffer = ByteBuffer.allocate(numberOfDecompressedBytes);
      compressorThreadLocal.get().decompress(compressedPointCloudByteBuffer, decompressedPointCloudByteBuffer, numberOfDecompressedBytes);
      decompressedPointCloudByteBuffer.flip();
      FloatBuffer decompressedPointCloudFloatBuffer = decompressedPointCloudByteBuffer.asFloatBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = decompressedPointCloudFloatBuffer.get();
         double y = decompressedPointCloudFloatBuffer.get();
         double z = decompressedPointCloudFloatBuffer.get();
         pointCoordinateConsumer.accept(i, x, y, z);
      }
   }

   public interface LidarPointCoordinateFunction
   {
      /**
       * @param pointIndex index of a point in the point-cloud
       * @param coordinateIndex x=0, y=1, z=2
       * @return
       */
      float getCoordinate(int pointIndex, int coordinateIndex);
   }

   public interface LidarPointConsumer
   {
      void accept(int index, double x, double y, double z);
   }
}
