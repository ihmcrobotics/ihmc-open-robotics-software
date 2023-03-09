package us.ihmc.perception.tools;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Arrays;
import java.util.HashMap;

public class PerceptionDebugTools
{
   public static void printMatches(String tag, PlanarRegionsList map, PlanarRegionsList regions, HashMap<Integer, TIntArrayList> matches, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("------------------------------------------------ Printing Matches ({}) ---------------------------------------------", tag);
      LogTools.info("Map Region Count: {}", map.getNumberOfPlanarRegions());
      LogTools.info("Incoming Regions Count: {}", regions.getNumberOfPlanarRegions());

      for (Integer key : matches.keySet())
      {
         int[] values = matches.get(key).toArray();
         LogTools.info("Match: ({}) -> {}", key, Arrays.toString(values));
      }
      LogTools.info("------------------------------------------------ Printing Matches End ---------------------------------------------");
   }

   public static void printPlanarRegionsListVertices(String tag, PlanarRegionsList regions, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("[{}]", tag);
      for(int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         LogTools.info("Region Index: {}, Region ID: {}", i, regions.getPlanarRegion(i).getRegionId());
         printPlanarRegionVertices(regions.getPlanarRegion(i), debug);
      }
   }

   public static void printPlanarRegionVertices(PlanarRegion region, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("Concave Hull Vertices -----------------------------------");
      for (int i = 0; i < region.getConcaveHullSize(); i++)
      {
         LogTools.info("[Region: {}] Point {} : {}", region.getRegionId(), i, region.getConcaveHullPoint3DInWorld(i));
      }
      System.out.println();
   }

   public static void printRegionIDs(String tag, PlanarRegionsList regions, boolean debug)
   {
      if (!debug)
         return;

      int[] ids = new int[regions.getNumberOfPlanarRegions()];
      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         ids[i] = regions.getPlanarRegion(i).getRegionId();
      }
      LogTools.info("[{}] Region IDs: {}", tag, Arrays.toString(ids));
   }

   public static void printPlane(PlanarRegion region, boolean debug)
   {
      if (!debug)
         return;

      LogTools.info("Plane: {}",
                    String.format("%.2f, %.2f, %.2f, %.2f",
                                  region.getNormal().getX(),
                                  region.getNormal().getY(),
                                  region.getNormal().getZ(),
                                  region.getNormal().dot(region.getPoint())));
   }

   public static void printTransform(String tag, RigidBodyTransform transform, boolean debug)
   {
      if (!debug)
         return;

      Point3D euler = new Point3D();
      transform.getRotation().getEuler(euler);

      LogTools.info("[{}] Translation: {}, Rotation: {}", tag, transform.getTranslation(), euler);
   }
}
