package us.ihmc.perception;

import us.ihmc.bytedeco.mapsenseWrapper.MapsenseWrapper.MapsenseExternal;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;

import java.awt.*;
import java.io.FileNotFoundException;

public class OusterRegionsCalculator
{
   /* Implementation: ihmc-perception/src/mapsense-wrapper/cpp/external/mapsense_external.cpp */
   private final MapsenseExternal mapsenseExternal;

   private int MAX_FLOATS = 400000;

   private final float[] points = new float[MAX_FLOATS];
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

   public void getPointCloud(RecyclingArrayList<Point3D32> pointArrayList)
   {
      for(int i = 0; i<MAX_FLOATS/3; i++)
      {
         pointArrayList.add().set(new Point3D32(points[i*3], points[i*3+1], points[i*3+2]));
      }
   }

   public void update()
   {
      long start = System.currentTimeMillis();
      mapsenseExternal.extractPlanarRegionsFromPointCloud(points, numPoints);
      long end = System.currentTimeMillis();

      System.out.println("Time Taken: " + (end - start) + "ms");
   }
}
