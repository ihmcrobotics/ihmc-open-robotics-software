package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.MapsenseWrapper.MapsenseExternal;
import us.ihmc.log.LogTools;

import java.io.FileNotFoundException;

public class OusterRegionsCalculator
{
   private final MapsenseExternal mapsenseExternal;
   private final float[] points = new float[400000];
   private int numPoints = 0;

   public OusterRegionsCalculator(MapsenseExternal mapsense)
   {
      mapsenseExternal = mapsense;

      try
      {
         numPoints = MapsenseTools.loadPointCloud("/home/bmishra/Workspace/Code/MapSense/Data/Extras/Clouds/Scan_94", points);
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      LogTools.info("Total Points Loaded: {}", numPoints);
   }

   public void update()
   {
      long start = System.currentTimeMillis();
      mapsenseExternal.extractPlanarRegionsFromPointCloud(points, numPoints);
      long end = System.currentTimeMillis();

      System.out.println("Time Taken: " + (end - start) + "ms");
   }
}
