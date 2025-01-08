package us.ihmc.sensors;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;

/**
 * Recorder for sensor poses. Uses a custom file format (.sensorposes)
 *    Records:
 *       - image sequence number
 *       - acquisition time (epoch second + nanos)
 *       - 3D position
 *       - quaternion orientation
 */
public class ImageSensorPoseRecorder
{
   private static final int BYTES_PER_FRAME = Long.BYTES + Long.BYTES + Integer.BYTES + (3 * Double.BYTES) + (4 * Double.BYTES);
   private static final int MAX_BUFFER_SIZE = BYTES_PER_FRAME * 512; // Max of 512 frames in the buffer

   private final Path file;
   private final ByteBuffer buffer = ByteBuffer.allocate(MAX_BUFFER_SIZE);

   public ImageSensorPoseRecorder(Path file)
   {
      this.file = file;
   }

   public void recordFrame(RawImage rawImage)
   {
      long sequenceNumber = rawImage.getSequenceNumber();
      Instant acquisitionTime = rawImage.getAcquisitionTime();
      Point3DBasics position = rawImage.getPosition();
      QuaternionBasics orientation = rawImage.getOrientation();

      // Pack sequence number
      buffer.putLong(sequenceNumber);
      // Pack acquisition time
      buffer.putLong(acquisitionTime.getEpochSecond());
      buffer.putInt(acquisitionTime.getNano());
      // Pack position
      buffer.putDouble(position.getX());
      buffer.putDouble(position.getY());
      buffer.putDouble(position.getZ());
      // Pack orientation
      buffer.putDouble(orientation.getX());
      buffer.putDouble(orientation.getY());
      buffer.putDouble(orientation.getZ());
      buffer.putDouble(orientation.getS());

      // If there is not enough space in the buffer for another frame, flush it to file
      if (buffer.remaining() < BYTES_PER_FRAME)
         flush();
   }

   /**
    * Writes the contents of the buffer to disk, then resets the buffer.
    * <p>
    * It is not necessary to call flush() every recordFrame call.
    * recordFrame will automatically flush when the buffer is full.
    */
   public void flush()
   {
      if (!Files.exists(file))
      {
         try
         {
            Files.createDirectories(file.getParent());
            Files.createFile(file);
         }
         catch (IOException e)
         {
            LogTools.error(e);
         }
      }

      buffer.flip();

      try (FileChannel channel = FileChannel.open(file, StandardOpenOption.WRITE, StandardOpenOption.APPEND))
      {
         channel.write(buffer);
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }

      buffer.clear();
   }
}
