package us.ihmc.communication.packets;

import perception_msgs.msg.dds.LidarScanMessage;
import gnu.trove.list.array.TByteArrayList;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.communication.compression.LZ4CompressionImplementation;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;

/**
 * This class should be used to create and unpack a {@link LidarScanMessage}.
 */
public class LidarPointCloudCompression
{
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = ThreadLocal.withInitial(LZ4CompressionImplementation::new);
   public static final double POINT_RESOLUTION = 0.003;
   private static final int maxPointCloudSize = 260000;
   private static final boolean debug = false;
   private static boolean hasPrintedStackTrace = false;

   public static void compressPointCloud(int pointCloudSize, LidarScanMessage messageToPack, LidarPointCoordinateFunction coordinateFunction)
   {
      // 3 coordinates per point, 4 bytes per coordinate
      int pointCloudSizeInBytes = Integer.BYTES * 3 * pointCloudSize;

      ByteBuffer byteBuffer = ByteBuffer.allocate(pointCloudSizeInBytes);
      IntBuffer pointCloudBuffer = byteBuffer.asIntBuffer();

      for (int i = 0; i < Math.min(pointCloudSize, maxPointCloudSize); i++)
      {
         for (int j = 0; j < 3; j++)
         {
            double coordinate = coordinateFunction.getCoordinate(i, j);
            int discritizedCoordinate = (int) (Math.round(coordinate / POINT_RESOLUTION));
            pointCloudBuffer.put(3 * i + j, discritizedCoordinate);
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
         if (!hasPrintedStackTrace)
         {
            hasPrintedStackTrace = true;
            e.printStackTrace();
         }
         return;
      }

      if (debug)
      {
         double compressionRatio = compressedPointCloudSize / ((double) pointCloudSizeInBytes);
         System.out.println(compressionRatio);
      }

      compressedPointCloudBuffer.flip();
      for (int i = 0; i < compressedPointCloudSize; i++)
         messageToPack.getScan().add(compressedPointCloudBuffer.get());

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
      IntBuffer decompressedPointCloudIntBuffer = decompressedPointCloudByteBuffer.asIntBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = POINT_RESOLUTION * decompressedPointCloudIntBuffer.get();
         double y = POINT_RESOLUTION * decompressedPointCloudIntBuffer.get();
         double z = POINT_RESOLUTION * decompressedPointCloudIntBuffer.get();
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
      double getCoordinate(int pointIndex, int coordinateIndex);
   }

   public interface LidarPointConsumer
   {
      void accept(int index, double x, double y, double z);
   }
}
