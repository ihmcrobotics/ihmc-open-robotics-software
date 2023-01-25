package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.ImageMat;
import us.ihmc.perception.ImageTools;

import java.io.FileNotFoundException;

public class MapsenseWrapperTest
{
   public static void main(String[] args) throws FileNotFoundException
   {
      BytedecoTools.loadMapsense();

      VisualOdometry.VisualOdometryExternal visualOdometry = new VisualOdometry.VisualOdometryExternal(500, 300);

      testBufferTransfer(visualOdometry);
      testImageTransferAndDisplay(visualOdometry);
   }

   public static void testBufferTransfer(VisualOdometry.VisualOdometryExternal visualOdometry)
   {
      float[] bufferA = new float[] {0.6f, 0.5f, 0.4f, 0.1f, 0.2f, 0.3f};

      long start = System.nanoTime();
      visualOdometry.printMat(bufferA, 2, 3);
      long end = System.nanoTime();
      System.out.println("Time Taken: " + (end - start) / 1000 + "us");
   }

   public static void testImageTransferAndDisplay(VisualOdometry.VisualOdometryExternal visualOdometry)
   {
      long start = System.nanoTime();

      ImageMat mat = ImageTools.loadAsImageMat("/home/bmishra/Workspace/Data/Datasets/dataset/sequences/00/image_2/000000.png");

      long ckpt1 = System.nanoTime();
      visualOdometry.displayMat(mat.getData(), mat.getRows(), mat.getCols(), 0);
      System.out.println("Time Taken (Loading): " + (ckpt1 - start) / 1000 + "us");
   }
}
