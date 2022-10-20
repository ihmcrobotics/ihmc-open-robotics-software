package us.ihmc.perception;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlanarRegionRegistration
{
   private int frameIndex = 0;
   private boolean modified = false;

   private PlanarRegionsList currentRegions;
   private PlanarRegionsList previousRegions;

   private String regionFilePath = "/home/quantum/Workspace/Code/MapSense/Data/Extras/Regions/Archive/Set_06_Circle/";
   ;

   public PlanarRegionRegistration()
   {
      previousRegions = new PlanarRegionsList();
      currentRegions = loadRegions(regionFilePath + "0000.txt", 0);
   }

   public void incrementIndex()
   {
      frameIndex += 20;
   }

   public void update()
   {
      String fileName = String.format("%1$4s", frameIndex).replace(' ', '0') + ".txt";
      LogTools.info("Loading File: {}", fileName);

      if (currentRegions.getNumberOfPlanarRegions() != 0)
      {
         previousRegions.clear();
         previousRegions.addPlanarRegionsList(currentRegions);
      }

      currentRegions = loadRegions(regionFilePath + fileName, frameIndex);

      HashMap<Integer, Integer> matches = PlanarRegionSLAMTools.findPlanarRegionMatches(previousRegions, currentRegions, 0.1f, 0.5f);

      LogTools.info("Previous: {} Current: {} Matches: {}",
                    previousRegions.getPlanarRegion(0).getConcaveHullSize(),
                    currentRegions.getPlanarRegion(0).getConcaveHullSize(),
                    matches.size());

      this.modified = true;
   }

   public static String getValue(BufferedReader reader, String key) throws IOException
   {
      String line = reader.readLine();
      //LogTools.info("Line: {}", line);
      String[] words;
      words = line.split(":");
      if (key.equals(words[0]))
      {
         return words[1];
      }
      else
         return null;
   }

   public static Point3D getPoint3D(BufferedReader reader, String key) throws IOException
   {
      String line = reader.readLine();
      //LogTools.info("Line (Point3D): {}", line);

      String[] words;
      String[] coordinates = null;
      words = line.split(":");

      if (key == null)
         coordinates = words[0].replace(" ", "").split(",");
      else if (words[0].equals(key))
         coordinates = words[1].replace(" ", "").split(",");

      if (coordinates != null)
         return new Point3D(Float.parseFloat(coordinates[0]), Float.parseFloat(coordinates[1]), Float.parseFloat(coordinates[2]));
      else
         return null;
   }

   public static PlanarRegionsList loadMapsensePlanarRegionsFromFile(File file,
                                                                     PolygonizerParameters polygonizerParameters,
                                                                     ConcaveHullFactoryParameters concaveHullFactoryParameters)
   {
      PlanarRegionsList listToReturn = null;
      List<PlanarRegionSegmentationRawData> planarRegionRawDataList = new ArrayList<>();

      String value = null;
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(file));

         int numRegions = 0;
         value = getValue(reader, "NumRegions");
         LogTools.info("NumRegions: {}", value);
         if (value != null)
         {
            numRegions = Integer.parseInt(value);
            for (int r = 0; r < numRegions; r++)
            {

               int regionId = Integer.parseInt(getValue(reader, "RegionID"));
               Point3D origin = getPoint3D(reader, "Center");
               Point3D normal = getPoint3D(reader, "Normal");

               AxisAngle orientation = new AxisAngle();
               orientation.set(EuclidGeometryTools.axisAngleFromZUpToVector3D(new Vector3D(normal)));
               RigidBodyTransform transformToWorld = new RigidBodyTransform(orientation, origin);

               List<Point3D> loadedPoints = new ArrayList<>();

               int numPatches = Integer.parseInt(getValue(reader, "NumPatches"));

               for (int patchIndex = 0; patchIndex < numPatches; patchIndex++)
               {
                  Point3D point = getPoint3D(reader, null);
                  loadedPoints.add(point);
               }

               PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(regionId, new Vector3D(normal), origin, loadedPoints);

               planarRegionRawDataList.add(data);
            }

            listToReturn = PlanarRegionPolygonizer.createPlanarRegionsList(planarRegionRawDataList, concaveHullFactoryParameters, polygonizerParameters);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      return listToReturn;
   }

   public PlanarRegionsList loadRegions(String path, int index)
   {
      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

      ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      ;
      concaveHullFactoryParameters.setEdgeLengthThreshold(0.224);
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(false);
      concaveHullFactoryParameters.setMaxNumberOfIterations(5000);
      concaveHullFactoryParameters.setTriangulationTolerance(0.0);

      PlanarRegionsList regions = loadMapsensePlanarRegionsFromFile(new File(path), polygonizerParameters, concaveHullFactoryParameters);

      for (PlanarRegion region : regions.getPlanarRegionsAsList())
         LogTools.info("Regions: {}", region.getConcaveHullSize());

      return regions;
   }

   public boolean modified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }

   public PlanarRegionsList getCurrentRegions()
   {
      return currentRegions;
   }
}
