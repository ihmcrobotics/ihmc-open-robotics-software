package us.ihmc.bytedeco.slamWrapper.test;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.perception.slamWrapper.FactorGraph;
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

      wrappedTest();
   }

   public static void wrappedTest()
   {
      FactorGraph factorGraph = new FactorGraph();

      Pose3D poseInitial = new Pose3D(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      Pose3D odometry = new Pose3D(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      Vector4D planeOne = new Vector4D(0.0f, 0.0f, 2.0f, 0.0f);
      Vector4D planeMeasOne = new Vector4D(0.0f, 0.0f, 2.0f, 0.0f);
      Vector4D planeMeasTwo = new Vector4D(0.0f, 0.0f, 1.0f, 0.0f);


      factorGraph.addPriorPoseFactor(1, poseInitial);
      factorGraph.addOdometryFactor(odometry, 2);

      factorGraph.addOrientedPlaneFactor(planeMeasOne, 1, 1);
      factorGraph.addOrientedPlaneFactor(planeMeasTwo, 1, 2);

      factorGraph.setPoseInitialValue(1, poseInitial);
      factorGraph.setPoseInitialValue(2, odometry);
      factorGraph.setOrientedPlaneInitialValue(1, planeOne);

      factorGraph.optimize();

      factorGraph.printResults();


   }

   public static void rawTest()
   {
      FactorGraph factorGraphExternal = new FactorGraph();

      //      factorGraphExternal.helloWorldTest();

      float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
      //
      //factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      //factorGraphExternal.addOdometryFactor(odometry, 2);
      //
      //factorGraphExternal.setPoseInitialValue(1, poseInitial);
      //factorGraphExternal.setPoseInitialValue(2, odometry);
      //
      //factorGraphExternal.optimize();
      //
      //factorGraphExternal.printResults();

      float[] planeOne = new float[]{0.0f, 0.0f, 2.0f, 0.0f};
      float[] planeMeasOne = new float[]{0.0f, 0.0f, 2.0f, 0.0f};
      float[] planeMeasTwo = new float[]{0.0f, 0.0f, 1.0f, 0.0f};

      factorGraphExternal.addPriorPoseFactor(1, poseInitial);
      factorGraphExternal.addOdometryFactor(odometry, 2);

      factorGraphExternal.addOrientedPlaneFactor(planeMeasOne, 1, 1);
      factorGraphExternal.addOrientedPlaneFactor(planeMeasTwo, 1, 2);

      factorGraphExternal.setPoseInitialValue(1, poseInitial);
      factorGraphExternal.setPoseInitialValue(2, odometry);
      factorGraphExternal.setOrientedPlaneInitialValue(1, planeOne);

      factorGraphExternal.optimize();

      factorGraphExternal.printResults();
   }
}
