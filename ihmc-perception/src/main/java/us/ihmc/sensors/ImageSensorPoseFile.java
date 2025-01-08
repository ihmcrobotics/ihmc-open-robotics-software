package us.ihmc.sensors;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.function.Consumer;

/**
 * Records:
 * - acquisition time (epoch second + nanos)
 * - 3D position
 * - quaternion orientation
 */
public class ImageSensorPoseFile
{
   private static final int BYTES_PER_FRAME = Long.BYTES + Integer.BYTES + (3 * Double.BYTES) + (4 * Double.BYTES);
   private static final int MAX_WRITE_BUFFER_SIZE = BYTES_PER_FRAME * 128; // Max of 128 frames in the write buffer

   private final Path file;
   private final ByteBuffer writeBuffer = ByteBuffer.allocate(MAX_WRITE_BUFFER_SIZE);
   private ByteBuffer readBuffer = ByteBuffer.allocate(0);

   public ImageSensorPoseFile(Path file)
   {
      this.file = file;
   }

   private void readFile()
   {
      try
      {
         if (readBuffer.capacity() != Files.size(file))
         {
            try (FileChannel channel = FileChannel.open(file, StandardOpenOption.READ))
            {
               readBuffer = ByteBuffer.allocate((int) channel.size());
               channel.read(readBuffer);
            }

            readBuffer.flip();
         }
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }
   }

   public int getNumberOfFrames()
   {
      readFile();

      return readBuffer.capacity() / BYTES_PER_FRAME;
   }

   public void readFrameData(long sequenceNumber, Consumer<Instant> instantConsumer, Point3DBasics position, QuaternionBasics orientation)
   {
      readFile();

      if (readBuffer != null)
      {
         readBuffer.position((int) (BYTES_PER_FRAME * sequenceNumber));

         long epochSecond = readBuffer.getLong();
         int nano = readBuffer.getInt();
         double x = readBuffer.getDouble();
         double y = readBuffer.getDouble();
         double z = readBuffer.getDouble();
         double q0 = readBuffer.getDouble();
         double q1 = readBuffer.getDouble();
         double q2 = readBuffer.getDouble();
         double q3 = readBuffer.getDouble();
         instantConsumer.accept(Instant.ofEpochSecond(epochSecond, nano));
         position.set(x, y, z);
         orientation.set(q0, q1, q2, q3);
      }
   }

   public void recordFrameData(Instant acquisitionTime, Point3DBasics position, QuaternionBasics orientation)
   {
      // Pack acquisition time
      writeBuffer.putLong(acquisitionTime.getEpochSecond());
      writeBuffer.putInt(acquisitionTime.getNano());
      // Pack position
      writeBuffer.putDouble(position.getX());
      writeBuffer.putDouble(position.getY());
      writeBuffer.putDouble(position.getZ());
      // Pack orientation
      writeBuffer.putDouble(orientation.getX());
      writeBuffer.putDouble(orientation.getY());
      writeBuffer.putDouble(orientation.getZ());
      writeBuffer.putDouble(orientation.getS());

      // If there is not enough space in the buffer for another frame, flush it to file
      if (writeBuffer.remaining() < BYTES_PER_FRAME)
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

      writeBuffer.flip();

      try (FileChannel channel = FileChannel.open(file, StandardOpenOption.WRITE, StandardOpenOption.APPEND))
      {
         channel.write(writeBuffer);
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }

      writeBuffer.clear();
   }
}
