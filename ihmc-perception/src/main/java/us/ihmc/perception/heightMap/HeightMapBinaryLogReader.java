package us.ihmc.perception.heightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.tuple3D.Point3D;

import java.io.Closeable;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;

public class HeightMapBinaryLogReader implements Closeable
{
   private final List<Long> frameOffsets = new ArrayList<>();
   private final FileChannel channel;
   private final Path filePath;

   public HeightMapBinaryLogReader(String path) throws IOException
   {
      this.filePath = Path.of(path);
      this.channel = FileChannel.open(filePath, StandardOpenOption.READ);
      indexFrames();
   }

   private void indexFrames() throws IOException
   {
      frameOffsets.clear();
      channel.position(0);

      ByteBuffer intBuffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);

      while (channel.position() < channel.size())
      {
         long frameStart = channel.position();

         // Read frame size
         intBuffer.clear();
         channel.read(intBuffer);
         intBuffer.flip();
         int frameSize = intBuffer.getInt();

         frameOffsets.add(frameStart);

         // Skip to next frame
         channel.position(frameStart + 4L + frameSize);
      }
   }

   public int getFrameCount()
   {
      return frameOffsets.size();
   }

   public HeightMapMessage loadFrame(int index) throws IOException
   {
      if (index < 0 || index >= frameOffsets.size())
         return null;

      long frameOffset = frameOffsets.get(index);

      // Read frame size
      ByteBuffer intBuffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
      channel.position(frameOffset);
      channel.read(intBuffer);
      intBuffer.flip();
      int frameSize = intBuffer.getInt();

      // Read entire frame bytes
      ByteBuffer frameBuffer = ByteBuffer.allocate(frameSize).order(ByteOrder.LITTLE_ENDIAN);
      channel.read(frameBuffer);
      frameBuffer.flip();

      double timestamp = frameBuffer.getDouble();

      int numFloats = (frameSize - 8) / Float.BYTES;

      float[] packedArray = new float[numFloats];
      frameBuffer.asFloatBuffer().get(packedArray);

      // Unpack header
      float widthInMeters = packedArray[0];
      float cellSizeInMeters = packedArray[1];
      float centerX = packedArray[2];
      float centerY = packedArray[3];
      float heightOffset = packedArray[4];
      float heightScaleFactor = packedArray[5];

      final int headerFloats = 6;
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      float[] heightsArray = new float[totalCells];
      System.arraycopy(packedArray, headerFloats, heightsArray, 0, totalCells);

      Mat mat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      FloatBuffer matBuffer = mat.createBuffer();
      matBuffer.put(heightsArray);
      matBuffer.rewind();

      Point3D center = new Point3D(centerX, centerY, 0.0);

      HeightMapMessage msg = new HeightMapMessage();
      HeightMapMessageTools.toMessage(mat, msg, center, widthInMeters, cellSizeInMeters);

      msg.setSequenceId(index + 2);

      return msg;
   }

   @Override
   public void close() throws IOException
   {
      channel.close();
   }
}
