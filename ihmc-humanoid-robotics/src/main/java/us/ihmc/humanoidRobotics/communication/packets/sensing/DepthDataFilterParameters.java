package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;

import us.ihmc.communication.packets.Packet;

public class DepthDataFilterParameters extends Packet<DepthDataFilterParameters>
{
   /*
    * Modifiable values with defaults
    */

   // data stores
   public boolean quadtree = true;
   public boolean octree = true;
   public boolean nearScan = true;

   // near scan config
   public float nearScanResolution = .01f;
   public int nearScanDecayMillis = 30000; // -1 to turn off decay
   public int nearScanCapacity = 100000; // -1 to turn off capacity

   public float nearScanRadius = 4.0f;
   public float nearScanZMinAboveFeet = -1.0f;
   public float nearScanZMaxAboveHead = 0.5f;
   public float nearScanRadians = (float) Math.PI * 3 / 4;

   public boolean nearScanCollisions = false;

   // global config
   public float minRange = 0.2f;
   public float maxRange = 5.0f;
   public float boundingBoxScale = 1.0f;

   // octree config
   public float octreeZMaxAboveHead = 0.0f;
   
   // quad tree config
   public float xCutoffPelvis = 0.0f;
   public float quadtreeHeightThreshold = 0.02f;
   public float quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2f;
   public int maxSameHeightPointsPerNode = 4;
   public double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
   
   public float quadTreeZAboveFeet = 0.25f;
   public float quadTreeZSlope = 0.5f;
   public float quadTreeZMax = 1.5f;
   
   public int maximumNumberOfPoints = maxSameHeightPointsPerNode * 75000;
   
   public static LinkedHashMap<String, DepthDataFilterParameters> presets = new LinkedHashMap<String, DepthDataFilterParameters>();
   static
   {
      presets.put("default", getDefaultParameters());
      loadSavedParameters();
   }

   /*
    * Static non-modifiable values
    */

   //TODO: 1cm looks really good, but kills the UI frames per second. 2.5cm better for that.
   public static final double GRID_RESOLUTION = 0.025; // in meters
   
   // Resolution Sphere
   public static final boolean USE_RESOLUTION_SPHERE = true;
   public static final double LIDAR_RESOLUTION_SPHERE_OUTER_RESOLUTION = 0.25;
   public static final double LIDAR_RESOLUTION_SPHERE_OUTER_RADIUS = 6.0;
   public static final double LIDAR_RESOLUTION_SPHERE_INNER_RESOLUTION = 0.02;
   public static final double LIDAR_RESOLUTION_SPHERE_INNER_RADIUS = 2;
   public static final double LIDAR_RESOLUTION_SPHERE_DISTANCE_FROM_HEAD = 1.0;
   public static final double OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE = 0.025;
   
   public static boolean LIDAR_ADJUSTMENT_ACTIVE = false;

   public DepthDataFilterParameters()
   {
   }

   public static DepthDataFilterParameters getDefaultParameters()
   {
      return new DepthDataFilterParameters();
   }

   private static final String fileName = "lidarFilterConfig.txt";

   public static void loadSavedParameters()
   {
      try
      {
//         BufferedReader reader = new BufferedReader(new FileReader(fileName));
         BufferedReader reader = new BufferedReader(new InputStreamReader(DepthDataFilterParameters.class.getClassLoader().getResourceAsStream(fileName),
               StandardCharsets.US_ASCII));
         DepthDataFilterParameters p = null;
         String name = "";
         String line;

         while ((line = reader.readLine()) != null)
         {
            if (p == null  && line.length() > 0)
            {
               name = line;
               p = new DepthDataFilterParameters();
            }
            else if (line.length() > 0)
            {
               String[] value = line.split("=");
               setParameter(p, value[0], value[1]);
            }
            else if (p != null)
            {
               presets.put(name, p);
               p = null;
            }
         }
         
         if (p != null)
         {
            presets.put(name, p);
         }
         
         reader.close();
      }
      catch (IOException e)
      {
//         e.printStackTrace();
         System.err.println("Could not load lidar config params!");
      }

   }

   private static void setParameter(DepthDataFilterParameters p, String parameter, String value)
   {
      if (parameter.equals("quadtree"))
         p.quadtree = Boolean.parseBoolean(value);
      else if (parameter.equals("octree"))
         p.octree = Boolean.parseBoolean(value);
      else if (parameter.equals("nearScan"))
         p.nearScan = Boolean.parseBoolean(value);

      else if (parameter.equals("nearScanResolution"))
         p.nearScanResolution = Float.parseFloat(value);
      else if (parameter.equals("nearScanDecayMillis"))
         p.nearScanDecayMillis = Integer.parseInt(value);
      else if (parameter.equals("nearScanCapacity"))
         p.nearScanCapacity = Integer.parseInt(value);

      else if (parameter.equals("nearScanRadius"))
         p.nearScanRadius = Float.parseFloat(value);
      else if (parameter.equals("nearScanZMinAboveFeet"))
         p.nearScanZMinAboveFeet = Float.parseFloat(value);
      else if (parameter.equals("nearScanZMaxAboveHead"))
         p.nearScanZMaxAboveHead = Float.parseFloat(value);
      else if (parameter.equals("nearScanRadians"))
         p.nearScanRadians = Float.parseFloat(value);

      else if (parameter.equals("nearScanCollisions"))
         p.nearScanCollisions = Boolean.parseBoolean(value);

      else if (parameter.equals("minRange"))
         p.minRange = Float.parseFloat(value);
      else if (parameter.equals("maxRange"))
         p.maxRange = Float.parseFloat(value);
      else if (parameter.equals("boundingBoxScale"))
         p.boundingBoxScale = Float.parseFloat(value);

      else if (parameter.equals("octreeZMaxAboveHead"))
         p.octreeZMaxAboveHead = Float.parseFloat(value);
      
      else if (parameter.equals("xCutoffPelvis"))
         p.xCutoffPelvis = Float.parseFloat(value);
      else if (parameter.equals("quadtreeHeightThreshold"))
         p.quadtreeHeightThreshold = Float.parseFloat(value);
      else if (parameter.equals("quadTreeMaxMultiLevelZChangeToFilterNoise"))
         p.quadTreeMaxMultiLevelZChangeToFilterNoise = Float.parseFloat(value);
      else if (parameter.equals("quadTreeZAboveFeet"))
         p.quadTreeZAboveFeet = Float.parseFloat(value);
      else if (parameter.equals("quadTreeZSlope"))
         p.quadTreeZSlope = Float.parseFloat(value);
      else if (parameter.equals("quadTreeZMax"))
         p.quadTreeZMax = Float.parseFloat(value);
      else if (parameter.equals("maximumNumberOfPoints"))
         p.maximumNumberOfPoints = Integer.parseInt(value);
      else
         throw new RuntimeException("invalid lidar filter parameter: " + parameter);
   }

   public static void saveParameters(String name, DepthDataFilterParameters p)
   {
      try
      {
         BufferedWriter writer = new BufferedWriter(new FileWriter("resources" + File.separator + fileName, true));
         
         writer.write("\n\n" + name + "\n");

         writer.write("quadtree="+p.quadtree+"\n");
         writer.write("octree="+p.octree+"\n");
         writer.write("nearScan="+p.nearScan+"\n");
         
         writer.write("nearScanResolution="+p.nearScanResolution+"\n");
         writer.write("nearScanDecayMillis="+p.nearScanDecayMillis+"\n");
         writer.write("nearScanCapacity="+p.nearScanCapacity+"\n");
         
         writer.write("nearScanRadius="+p.nearScanRadius+"\n");
         writer.write("nearScanZMinAboveFeet="+p.nearScanZMinAboveFeet+"\n");
         writer.write("nearScanZMaxAboveHead="+p.nearScanZMaxAboveHead+"\n");
         writer.write("nearScanRadians="+p.nearScanRadians+"\n");
         
         writer.write("nearScanCollisions="+p.nearScanCollisions+"\n");
         
         writer.write("minRange="+p.minRange+"\n");
         writer.write("maxRange="+p.maxRange+"\n");
         writer.write("boundingBoxScale="+p.boundingBoxScale+"\n");

         writer.write("octreeZMaxAboveHead="+p.octreeZMaxAboveHead+"\n");
         
         writer.write("xCutoffPelvis="+p.xCutoffPelvis+"\n");
         writer.write("quadtreeHeightThreshold="+p.quadtreeHeightThreshold+"\n");
         writer.write("quadTreeMaxMultiLevelZChangeToFilterNoise="+p.quadTreeMaxMultiLevelZChangeToFilterNoise+"\n");
         writer.write("quadTreeZAboveFeet="+p.quadTreeZAboveFeet+"\n");
         writer.write("quadTreeZSlope="+p.quadTreeZSlope+"\n");
         writer.write("quadTreeZMax="+p.quadTreeZMax+"\n");
         writer.write("maximumNumberOfPoints="+p.maximumNumberOfPoints+"\n");
         
         writer.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   @Override
   public boolean epsilonEquals(DepthDataFilterParameters other, double epsilon)
   {
      boolean same = true;
      same &= quadtree == other.quadtree;
      same &= octree == other.octree;
      same &= nearScan == other.nearScan;

      same &= Math.abs(nearScanResolution - other.nearScanResolution) < epsilon;
      same &= nearScanDecayMillis == other.nearScanDecayMillis;
      same &= nearScanCapacity == other.nearScanCapacity;

      same &= Math.abs(nearScanRadius - other.nearScanRadius) < epsilon;
      same &= Math.abs(nearScanZMaxAboveHead - other.nearScanZMaxAboveHead) < epsilon;
      same &= Math.abs(nearScanZMinAboveFeet - other.nearScanZMinAboveFeet) < epsilon;
      same &= Math.abs(nearScanRadians - other.nearScanRadians) < epsilon;

      return same;
   }
}
