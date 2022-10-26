package us.ihmc.bytedeco.mapsenseWrapper.test;

import us.ihmc.bytedeco.mapsenseWrapper.MapsenseWrapper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MapsenseTools;
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
      BytedecoTools.loadMapsenseLibraries();

      MapsenseWrapper.MapsenseExternal mapsenseExternal = new MapsenseWrapper.MapsenseExternal();

//      float[] poseInitial = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//      float[] odometry = new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};

//      mapsenseExternal.printMat(poseInitial, 6, 1);
//      mapsenseExternal.loadMat();

      float[] points = new float[400000];
      int numPoints = MapsenseTools.loadPointCloud("/home/quantum/Workspace/Code/MapSense/Data/Extras/Clouds/Scan_94", points);
      LogTools.info("Total Points Loaded: {}", numPoints);

      long start = System.currentTimeMillis();
      mapsenseExternal.extractPlanarRegionsFromPointCloud(points, numPoints);
      long end = System.currentTimeMillis();
      System.out.println("Time Taken: " +
                         (end - start) + "ms");

      System.out.println("Reached");
   }

}
