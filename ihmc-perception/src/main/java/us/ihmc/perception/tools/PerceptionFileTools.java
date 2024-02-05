package us.ihmc.perception.tools;

import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
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
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PerceptionFileTools
{
   private static String getColonSeparatedValueWithKey(BufferedReader reader, String key) throws IOException
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
         value = getColonSeparatedValueWithKey(reader, "NumRegions");
         LogTools.info("NumRegions: {}", value);
         if (value != null)
         {
            numRegions = Integer.parseInt(value);
            for (int r = 0; r < numRegions; r++)
            {

               int regionId = Integer.parseInt(getColonSeparatedValueWithKey(reader, "RegionID"));
               Point3D origin = getPoint3D(reader, "Center");
               Point3D normal = getPoint3D(reader, "Normal");

               AxisAngle orientation = new AxisAngle();
               orientation.set(EuclidGeometryTools.axisAngleFromZUpToVector3D(new Vector3D(normal)));
               RigidBodyTransform transformToWorld = new RigidBodyTransform(orientation, origin);

               List<Point3D> loadedPoints = new ArrayList<>();

               int numPatches = Integer.parseInt(getColonSeparatedValueWithKey(reader, "NumPatches"));

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

   public static Mat loadImage(String filePath)
   {
      Mat bgrInputImage = opencv_imgcodecs.imread(filePath);
      return bgrInputImage;
   }
}
