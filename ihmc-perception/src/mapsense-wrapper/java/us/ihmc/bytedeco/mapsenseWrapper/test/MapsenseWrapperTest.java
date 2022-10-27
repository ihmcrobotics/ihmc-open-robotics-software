package us.ihmc.bytedeco.mapsenseWrapper.test;

import us.ihmc.bytedeco.mapsenseWrapper.VisualOdometry;
import us.ihmc.perception.BytedecoTools;

import java.io.FileNotFoundException;
import java.util.Arrays;

public class MapsenseWrapperTest
{
   public static void main(String[] args) throws FileNotFoundException
   {
      BytedecoTools.loadMapsenseLibraries();

      VisualOdometry.VisualOdometryExternal visualOdometry = new VisualOdometry.VisualOdometryExternal();

      float[] bufferA = new float[] {0.6f, 0.5f, 0.4f, 0.1f, 0.2f, 0.3f};

      long start = System.nanoTime();
      visualOdometry.printMat(bufferA, 2,3);
      long end = System.nanoTime();
      System.out.println("Time Taken: " + (end - start) / 1000 + "us");

   }
}
