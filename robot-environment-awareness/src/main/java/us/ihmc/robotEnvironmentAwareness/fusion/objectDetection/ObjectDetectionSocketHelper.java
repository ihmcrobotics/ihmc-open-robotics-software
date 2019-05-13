package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferInt;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;

public class ObjectDetectionSocketHelper
{
   /**
    * ObjectType, xmin, xmax, ymin, ymax.
    */
   public static int[] convertStringToIntArray(String msgFromPython)
   {
      int[] intArray = Arrays.stream(msgFromPython.split(",")).mapToInt(Integer::parseInt).toArray();
      return intArray;
   }

   public static byte[] convertImgToBytes(BufferedImage bufferedImage)
   {
      DataBuffer dataBuffer = bufferedImage.getData().getDataBuffer();
      int[] imageInts = ((DataBufferInt) dataBuffer).getData();
      ByteBuffer byteBuffer = ByteBuffer.allocate(imageInts.length * 4);
      IntBuffer intBuffer = byteBuffer.asIntBuffer();
      intBuffer.put(imageInts);
      byte[] imageBytes = byteBuffer.array();
      return imageBytes;
   }

   public static byte[] convertImgDimToBytes(int size, int width, int height)
   {
      int[] imgDim = {size, width, height};
      ByteBuffer byteBuffer = ByteBuffer.allocate(imgDim.length * 4);
      IntBuffer intBuffer = byteBuffer.asIntBuffer();
      intBuffer.put(imgDim);
      byte[] imageSizeBytes = byteBuffer.array();
      return imageSizeBytes;
   }
}