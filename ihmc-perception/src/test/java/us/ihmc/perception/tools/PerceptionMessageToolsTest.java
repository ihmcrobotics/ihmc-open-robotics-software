package us.ihmc.perception.tools;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import perception_msgs.ImageMessage;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;

import java.nio.ByteBuffer;

import static org.junit.jupiter.api.Assertions.*;

public class PerceptionMessageToolsTest
{
   @Test
   public void testPackDataArrayRespectsPreAllocatedBytePointerLimit()
   {
      Mat image = new Mat(64, 64, opencv_core.CV_8UC3);
      image.data().fill((byte) 127);

      BytePointer preAllocatedPointer = new BytePointer(OpenCVTools.dataSize(image));
      opencv_imgcodecs.imencode(".jpg", image, preAllocatedPointer, OpenCVTools.compressionParametersJPG);

      int encodedByteCount = (int) (preAllocatedPointer.limit() - preAllocatedPointer.position());
      assertTrue(encodedByteCount > 0);
      assertTrue(encodedByteCount < OpenCVTools.dataSize(image), "JPEG should be smaller than raw image allocation");

      ImageMessage message = new ImageMessage();
      PerceptionMessageTools.packImageMessageData(message, preAllocatedPointer);

      assertEquals(encodedByteCount, message.getData().size());

      preAllocatedPointer.close();
      image.close();
   }

   @Test
   public void testImageMessageCDRRoundtripWithPreAllocatedJpegPointer()
   {
      Mat image = new Mat(32, 32, opencv_core.CV_8UC3);
      image.data().fill((byte) 42);

      BytePointer jpegData = new BytePointer(OpenCVTools.dataSize(image));
      opencv_imgcodecs.imencode(".jpg", image, jpegData, OpenCVTools.compressionParametersJPG);

      ImageMessage originalMessage = new ImageMessage();
      originalMessage.setImageWidth((short) image.cols());
      originalMessage.setImageHeight((short) image.rows());
      originalMessage.setPixelFormat(PixelFormat.BGR8.toByte());
      originalMessage.setCompressionType(CompressionType.JPEG.toByte());
      PerceptionMessageTools.packImageMessageData(originalMessage, jpegData);

      ImageMessage deserializedMessage = roundTripThroughCdr(originalMessage);

      assertEquals(originalMessage.getData().size(), deserializedMessage.getData().size());
      assertEquals(PixelFormat.BGR8.toByte(), deserializedMessage.getPixelFormat());
      assertEquals(CompressionType.JPEG.toByte(), deserializedMessage.getCompressionType());
      assertArrayEquals(getDataBytes(originalMessage), getDataBytes(deserializedMessage));

      jpegData.close();
      image.close();
   }

   @Test
   public void testImageMessageCDRRoundtripWithDepthPngPointer()
   {
      Mat depthImage = new Mat(120, 160, opencv_core.CV_16UC1);
      depthImage.data().fill((byte) 0);

      BytePointer pngData = new BytePointer();
      OpenCVTools.compressImagePNG(depthImage, pngData);

      ImageMessage originalMessage = new ImageMessage();
      originalMessage.setImageWidth((short) depthImage.cols());
      originalMessage.setImageHeight((short) depthImage.rows());
      originalMessage.setPixelFormat(PixelFormat.GRAY16.toByte());
      originalMessage.setCompressionType(CompressionType.PNG.toByte());
      PerceptionMessageTools.packImageMessageData(originalMessage, pngData);

      ImageMessage deserializedMessage = roundTripThroughCdr(originalMessage);

      assertEquals(originalMessage.getData().size(), deserializedMessage.getData().size());
      assertEquals(PixelFormat.GRAY16.toByte(), deserializedMessage.getPixelFormat());
      assertEquals(CompressionType.PNG.toByte(), deserializedMessage.getCompressionType());
      assertEquals(0, deserializedMessage.getOusterBeamAltitudeAngles().size());
      assertEquals(0, deserializedMessage.getOusterBeamAzimuthAngles().size());
      assertArrayEquals(getDataBytes(originalMessage), getDataBytes(deserializedMessage));

      pngData.close();
      depthImage.close();
   }

   private static ImageMessage roundTripThroughCdr(ImageMessage originalMessage)
   {
      CDRBuffer buffer = new CDRBuffer();
      int payloadSize = CDRBuffer.PAYLOAD_HEADER.length + originalMessage.calculateSizeBytes(0);
      buffer.ensureRemainingCapacity(payloadSize);
      buffer.rewind();
      buffer.writePayloadHeader();
      originalMessage.serialize(buffer);
      assertEquals(payloadSize, buffer.getBufferUnsafe().position());

      ByteBuffer byteBuffer = buffer.getBufferUnsafe();
      byte[] bytes = new byte[payloadSize];
      byteBuffer.position(0);
      byteBuffer.get(bytes);

      CDRBuffer readBuffer = new CDRBuffer();
      readBuffer.ensureRemainingCapacity(bytes.length);
      readBuffer.getBufferUnsafe().put(bytes);
      readBuffer.rewind();
      readBuffer.readPayloadHeader();

      ImageMessage deserializedMessage = new ImageMessage();
      deserializedMessage.deserialize(readBuffer);
      return deserializedMessage;
   }

   private static byte[] getDataBytes(ImageMessage message)
   {
      ByteBuffer dataBuffer = message.getData().getBuffer();
      byte[] bytes = new byte[message.getData().size()];
      dataBuffer.position(0);
      dataBuffer.get(bytes, 0, bytes.length);
      return bytes;
   }
}
