package us.ihmc.bytedeco.slamWrapper.test;

import us.ihmc.bytedeco.slamWrapper.SlamWrapper;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class SlamWrapperTest
{
   private static void loadLibraries() throws IOException
   {
      List<String> libraryFiles = new ArrayList<>();
      libraryFiles.add("libtbb.so");

      libraryFiles.add("libboost_filesystem.so");
      libraryFiles.add("libboost_chrono.so");
      libraryFiles.add("libboost_timer.so");
      libraryFiles.add("libboost_serialization.so");

      libraryFiles.add("libmetis-gtsam.so");
      libraryFiles.add("libgtsam.so");
      libraryFiles.add("libslam-wrapper.so");
      libraryFiles.add("libjniSlamWrapper.so");

      WorkspaceDirectory resourcesDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/slam-wrapper/resources");
      for (String libraryFile : libraryFiles)
      {
         System.load(new WorkspaceFile(resourcesDirectory, libraryFile).getFilePath().toAbsolutePath().normalize().toString());
      }
   }

   public static void main(String[] args)
   {
      try
      {
         loadLibraries();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      SlamWrapper.FactorGraphExternal factorGraphExternal = new SlamWrapper.FactorGraphExternal();

//      factorGraphExternal.helloWorldTest();

      float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};

      factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      factorGraphExternal.addOdometryFactor(odometry, 2);

      factorGraphExternal.setPoseInitialValue(1, poseInitial);
      factorGraphExternal.setPoseInitialValue(2, odometry);

      factorGraphExternal.optimize();

      factorGraphExternal.printResults();
   }
}
