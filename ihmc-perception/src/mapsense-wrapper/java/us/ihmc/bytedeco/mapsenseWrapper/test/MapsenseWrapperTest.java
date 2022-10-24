package us.ihmc.bytedeco.mapsenseWrapper.test;

import us.ihmc.bytedeco.mapsenseWrapper.MapsenseWrapper;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class MapsenseWrapperTest
{
   public static void main(String[] args) throws FileNotFoundException
   {
      try
      {
         loadLibraries();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      MapsenseWrapper.MapsenseExternal mapsenseExternal = new MapsenseWrapper.MapsenseExternal();

      //float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      //float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};

      //mapsenseExternal.printMat(poseInitial, 6, 1);
      //mapsenseExternal.loadMat();

      float[] points = new float[400000];
      int numPoints = loadPointCloud("/home/quantum/Workspace/Code/MapSense/Data/Extras/Clouds/Scan_94", points);
      
      mapsenseExternal.extractPlanarRegionsFromPointCloud(points, numPoints);

   }

   private static int loadPointCloud(String filePath, float[] pointsToPack) throws FileNotFoundException
   {
      Scanner scanner = new Scanner(new File(filePath));

      scanner.nextLine();
      int i = 0;
      while(scanner.hasNext())
      {
         String input = scanner.nextLine();
         String[] words = input.split(" ");

         System.out.println("Index:" + i + ":\t" + Arrays.toString(words));

         pointsToPack[i*3] = Float.parseFloat(words[0]);
         pointsToPack[i*3+1] = Float.parseFloat(words[1]);
         pointsToPack[i*3+2] = Float.parseFloat(words[2]);

         i++;

      }
      return i;
   }

   private static void loadLibraries() throws IOException
   {
      // We need to disable javacpp from trying to automatically load libraries.
      // Otherwise, it will try to load them by name when they aren't in the library path
      // (LD_LIBRARY_PATH on Linux).
      //
      // The approach taken here is to use System.load to load each library by explicit
      // absolute path on disk.
      System.setProperty("org.bytedeco.javacpp.loadlibraries", "false");

      List<String> libraryFiles = new ArrayList<>();

      libraryFiles.add("libmapsense-wrapper.so");
      libraryFiles.add("libjniMapsenseWrapper.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/mapsense-wrapper/resources");
      for (String libraryFile : libraryFiles)
      {
         System.load(new WorkspaceFile(resourcesDirectory, libraryFile).getFilePath().toAbsolutePath().normalize().toString());
      }
   }
}
