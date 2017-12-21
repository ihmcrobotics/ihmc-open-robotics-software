package us.ihmc.robotics;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionFileTools
{
   /**
    * Generates a default timestamped name that can be used to generate automated and unique
    * folders.
    * 
    * @return a {@code String} of the form: "20171201_163422_PlanarRegion".
    */
   public static String createDefaultTimeStampedFolderName()
   {
      return getDate() + "_PlanarRegion";
   }

   /**
    * Generates a {@code String} representing the data & time as of right now, like just right now.
    * 
    * @return the data & time as a {@code String}.
    */
   public static String getDate()
   {
      DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");
      return LocalDateTime.now().format(dateTimeFormatter);
   }

   /**
    * Creates a directory with the given path and export into files the data contained in
    * {@code planarRegionData}.
    * 
    * @param folderPath the path of the folder that will contain the exported files.
    * @param planarRegionData the planar regions to be exported to files. Not modified.
    * @return whether the exportation succeeded or not.
    */
   public static boolean exportPlanarRegionData(Path folderPath, PlanarRegionsList planarRegionData)
   {
      try
      {
         Files.createDirectories(folderPath);
         writePlanarRegionsData(folderPath, planarRegionData);
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   public static boolean isPlanarRegionFile(File file)
   {
      if (file == null || !file.exists() || !file.isDirectory())
         return false;

      try
      {
         return importPlanRegionDataInternal(file) != null;
      }
      catch (IOException e)
      {
         return false;
      }
   }

   /**
    * Load from the given data folder planar region data that has been previously exported via
    * {@link #exportPlanarRegionData(Path, PlanarRegionsList)}.
    * 
    * @param dataFolder the data folder containing the files with the planar region data.
    * @return the planar regions if succeeded, {@code null} otherwise.
    */
   public static PlanarRegionsList importPlanRegionData(File dataFolder)
   {

      try
      {
         PlanarRegionsList loadedRegions = importPlanRegionDataInternal(dataFolder);
         if (loadedRegions == null)
            PrintTools.error(PlanarRegionFileTools.class, "Could not load the file: " + dataFolder.getName());
         return loadedRegions;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private static PlanarRegionsList importPlanRegionDataInternal(File dataFolder) throws IOException
   {
      File headerFile = new File(dataFolder, "header.txt");
      List<PlanarRegion> planarRegions = new ArrayList<>();

      FileReader fileReader = new FileReader(headerFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         line = line.replaceAll("regionId: ", "");
         line = line.replaceAll("index: ", "");
         line = line.replaceAll("origin: ", "");
         line = line.replaceAll("normal: ", "");
         line = line.replaceAll("\\[", "");
         line = line.replaceAll("\\]", "");
         line = line.replaceAll("concave hull size: ", "");
         line = line.replaceAll("number of convex polygons: ", "");
         line = line.replaceAll(" ", "");
         String[] values = line.split(cvsSplitBy);

         int i = 0;

         int regionId = Integer.parseInt(values[i++]);
         int regionIndex = Integer.parseInt(values[i++]);
         String fileName = "region" + regionId + "_" + regionIndex;

         float xOrigin = Float.parseFloat(values[i++]);
         float yOrigin = Float.parseFloat(values[i++]);
         float zOrigin = Float.parseFloat(values[i++]);
         Point3D origin = new Point3D(xOrigin, yOrigin, zOrigin);

         float xNormal = Float.parseFloat(values[i++]);
         float yNormal = Float.parseFloat(values[i++]);
         float zNormal = Float.parseFloat(values[i++]);
         Vector3D normal = new Vector3D(xNormal, yNormal, zNormal);

         int concaveHullSize = Integer.parseInt(values[i++]);
         int numberOfConvexPolygons = Integer.parseInt(values[i++]);

         int[] convexPolygonsSize = new int[numberOfConvexPolygons];

         for (int hullIndex = 0; hullIndex < numberOfConvexPolygons; hullIndex++)
            convexPolygonsSize[hullIndex] = Integer.parseInt(values[i++]);

         PlanarRegion loadedRegion = loadPlanarRegionVertices(dataFolder, fileName, concaveHullSize, convexPolygonsSize, regionId, origin, normal);
         if (loadedRegion != null)
         {
            planarRegions.add(loadedRegion);
         }
         else
         {
            bufferedReader.close();
            return null;
         }
      }

      bufferedReader.close();

      return new PlanarRegionsList(planarRegions);
   }

   private static void writePlanarRegionsData(Path folderPath, PlanarRegionsList planarRegionData) throws IOException
   {
      File header = new File(folderPath.toFile(), "header.txt");
      FileWriter fileWriter = new FileWriter(header);

      Map<Integer, MutableInt> regionIdToIndex = new HashMap<>();

      for (PlanarRegion region : planarRegionData.getPlanarRegionsAsList())
      {
         Point3D origin = new Point3D();
         Vector3D normal = new Vector3D();
         region.getPointInRegion(origin);
         region.getNormal(normal);

         int numberOfConvexPolygons = region.getNumberOfConvexPolygons();

         int[] convexPolygonsSizes = new int[numberOfConvexPolygons];
         for (int i = 0; i < numberOfConvexPolygons; i++)
            convexPolygonsSizes[i] = region.getConvexPolygon(i).getNumberOfVertices();

         int regionId = region.getRegionId();
         MutableInt regionIndex = regionIdToIndex.getOrDefault(regionId, new MutableInt(0));
         regionIdToIndex.put(regionId, regionIndex);
         regionIndex.increment();

         fileWriter.write("regionId: " + Integer.toString(regionId));
         fileWriter.write(", index: " + Integer.toString(regionIndex.getValue().intValue()));
         fileWriter.write(", origin: " + origin.getX() + ", " + origin.getY() + ", " + origin.getZ());
         fileWriter.write(", normal: " + normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
         fileWriter.write(", concave hull size: " + region.getConcaveHullSize());
         fileWriter.write(", number of convex polygons: " + numberOfConvexPolygons + ", " + Arrays.toString(convexPolygonsSizes));

         fileWriter.write("\n");

         writePlanarRegionVertices(folderPath, region, regionIndex.intValue());
      }

      fileWriter.close();
   }

   private static void writePlanarRegionVertices(Path folderPath, PlanarRegion region, int regionIndex) throws IOException
   {
      File regionFile = new File(folderPath.toFile(), "region" + region.getRegionId() + "_" + regionIndex);
      FileWriter fileWriter = new FileWriter(regionFile);

      for (Point2D vertex : region.getConcaveHull())
      {
         fileWriter.write(vertex.getX() + ", " + vertex.getY() + "\n");
      }

      for (int polygonIndex = 0; polygonIndex < region.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = region.getConvexPolygon(polygonIndex);

         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            Point2DReadOnly vertex = convexPolygon.getVertex(vertexIndex);
            fileWriter.write(vertex.getX() + ", " + vertex.getY() + "\n");
         }
      }

      fileWriter.close();
   }

   private static PlanarRegion loadPlanarRegionVertices(File dataFolder, String regionFileName, int concaveHullSize, int[] convexPolygonsSize, int regionId,
                                                        Point3D origin, Vector3D normal)
   {
      try
      {
         File regionFile = new File(dataFolder, regionFileName);
         FileReader fileReader = new FileReader(regionFile);
         BufferedReader bufferedReader = new BufferedReader(fileReader);

         String line = "";
         String cvsSplitBy = ",";

         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);
         RigidBodyTransform transformToWorld = new RigidBodyTransform(orientation, origin);

         List<Point2D> loadedPoints = new ArrayList<>();

         while ((line = bufferedReader.readLine()) != null)
         {
            String[] coordsAsString = line.split(cvsSplitBy);
            float x = Float.parseFloat(coordsAsString[0]);
            float y = Float.parseFloat(coordsAsString[1]);
            loadedPoints.add(new Point2D(x, y));
         }

         bufferedReader.close();

         Point2D[] concaveHullVertices = new Point2D[concaveHullSize];

         for (int i = 0; i < concaveHullSize; i++)
         {
            concaveHullVertices[i] = loadedPoints.remove(0);
         }

         List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

         for (int polygonIndex = 0; polygonIndex < convexPolygonsSize.length; polygonIndex++)
         {
            int convexPolygonSize = convexPolygonsSize[polygonIndex];
            ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

            for (int i = 0; i < convexPolygonSize; i++)
            {
               convexPolygon.addVertex(loadedPoints.remove(0));
            }

            convexPolygon.update();
            convexPolygons.add(convexPolygon);
         }
         PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullVertices, convexPolygons);
         planarRegion.setRegionId(regionId);
         return planarRegion;
      }
      catch (IOException e)
      {
         return null;
      }
   }
}
