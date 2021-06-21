package us.ihmc.robotics;

import java.io.*;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;

import org.apache.commons.io.IOUtils;
import org.apache.commons.lang3.mutable.MutableInt;

import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.math3.util.Pair;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
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
    * @param folderPath       the path of the folder that will contain the exported files.
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

   /**
    * Creates a single file with the given path and export the data contained in
    * {@code planarRegionData}.
    *
    * @param filePath         the path of the file that will contain the exported data.
    * @param planarRegionData the planar regions to be exported. Not modified.
    * @return whether the exportation succeeded or not.
    */
   public static boolean exportPlanarRegionDataAsFile(Path filePath, PlanarRegionsList planarRegionData)
   {
      try
      {
         Files.createFile(filePath);
         FileOutputStream ostream = new FileOutputStream(filePath.toFile());
         writePlanarRegionsDataToStream(ostream, planarRegionData);
         ostream.close();
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   /**
    * Creates a single file with the given path and export the data contained in
    * {@code planarRegionData}.
    *
    * @param ostream          the stream to which the data should be written.
    * @param planarRegionData the planar regions to be exported. Not modified.
    * @return whether the exportation succeeded or not.
    */
   public static boolean exportPlanarRegionDataToStream(OutputStream ostream, PlanarRegionsList planarRegionData)
   {
      try
      {
         writePlanarRegionsDataToStream(ostream, planarRegionData);
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
      if (file == null || !file.exists())
         return false;

      try
      {
         if (file.isDirectory())
            return importPlanarRegionDataInternal(filename -> new File(file, filename)) != null;
         else
            return importPlanarRegionDataFromFileInternal(file) != null;
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
    * @param data the data folder or file containing the planar region data.
    * @return the planar regions if succeeded, {@code null} otherwise.
    */
   public static PlanarRegionsList importPlanarRegionData(File data)
   {
      try
      {
         PlanarRegionsList loadedRegions;
         if (data.isDirectory())
            loadedRegions = importPlanarRegionDataInternal(filename -> new File(data, filename));
         else
            loadedRegions = importPlanarRegionDataFromFileInternal(data);
         if (loadedRegions == null)
            LogTools.error("Could not load the file: " + data.getName());
         return loadedRegions;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   /**
    * Load from the given data folder planar region data that has been previously exported via
    * {@link #exportPlanarRegionData(Path, PlanarRegionsList)}.
    * <p>
    * When calling this method, dataFolder is expressed relative to the package name of the the given Class.
    *
    * @param clazz      the class which has the resource folder.
    * @param dataFolder the name of data folder containing the files with the planar region data.
    * @return the planar regions if succeeded, {@code null} otherwise.
    */
   public static PlanarRegionsList importPlanarRegionData(Class<?> clazz, String dataFolder)
   {
      try
      {
         PlanarRegionsList loadedRegions = importPlanarRegionDataInternalForTests(filename ->
                                                                                  {
                                                                                     String resourceName = dataFolder + "/" + filename;
                                                                                     InputStream inputStream = clazz.getResourceAsStream(resourceName);
                                                                                     if (inputStream == null)
                                                                                     {
                                                                                        LogTools.error("Could not open resource: " + resourceName,
                                                                                                       PlanarRegionsList.class);
                                                                                        return null;
                                                                                     }
                                                                                     return new BufferedReader(new InputStreamReader(inputStream));
                                                                                  });
         if (loadedRegions == null)
            LogTools.error("Could not load the file: " + dataFolder);
         return loadedRegions;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   /**
    * Load from the given data folder planar region data that has been previously exported via
    * {@link #exportPlanarRegionData(Path, PlanarRegionsList)}.
    * <p>
    * When calling this method, dataFolder should be the fully qualified name of the resource, e.g.
    * /us/ihmc/.../planarRegions/
    *
    * @param classLoader the classLoader for loading the planarRegionsList
    * @param dataFolder  the fully qualified name of data folder containing the files with the planar region data.
    * @return the planar regions if succeeded, {@code null} otherwise.
    */
   public static PlanarRegionsList importPlanarRegionData(ClassLoader classLoader, String dataFolder)
   {
      try
      {
         PlanarRegionsList loadedRegions = importPlanarRegionDataInternalForTests(filename ->
                                                                                  {
                                                                                     String resourceName = dataFolder + "/" + filename;
                                                                                     InputStream inputStream = classLoader.getResourceAsStream(resourceName);
                                                                                     if (inputStream == null)
                                                                                     {
                                                                                        LogTools.error("Could not open resource: " + resourceName,
                                                                                                       PlanarRegionsList.class);
                                                                                        return null;
                                                                                     }
                                                                                     return new BufferedReader(new InputStreamReader(inputStream));
                                                                                  });
         if (loadedRegions == null)
            LogTools.error("Could not load the file: " + dataFolder);
         return loadedRegions;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   /**
    * Load from the given data folder planar region data that has been previously exported via
    * {@link #exportPlanarRegionData(Path, PlanarRegionsList)}.
    *
    * @param dataFolderRelativePath the data folder containing the files with the planar region data.
    * @return the planar regions if succeeded, {@code null} otherwise.
    */
   public static PlanarRegionsList importPlanarRegionData(Class<?> loadingClass, Path dataFolderRelativePath)
   {
      try
      {
         PlanarRegionsList loadedRegions = importPlanarRegionDataInternal(filename -> fileFromClassPath(loadingClass,
                                                                                                        Paths.get(dataFolderRelativePath.toString(),
                                                                                                                  filename)));
         if (loadedRegions == null)
            LogTools.error("Could not load the file: " + dataFolderRelativePath.toString());
         return loadedRegions;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private static PlanarRegionsList importPlanarRegionDataInternal(FileCreator fileCreator) throws IOException
   {
      File headerFile = fileCreator.createFile("header.txt");
      List<PlanarRegion> planarRegions = new ArrayList<>();

      FileReader fileReader = new FileReader(headerFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);

      while (true)
      {
         Point3D origin = new Point3D();
         Vector3D normal = new Vector3D();
         MutableInt regionId = new MutableInt();
         MutableInt concaveHullSize = new MutableInt();
         TIntArrayList convexPolygonsSize = new TIntArrayList();

         String fileName = readHeaderLine(bufferedReader, origin, normal, regionId, concaveHullSize, convexPolygonsSize);

         if (fileName == null)
            break;

         File regionFile = fileCreator.createFile(fileName);
         FileReader regionFileReader = new FileReader(regionFile);
         BufferedReader regionBufferedReader = new BufferedReader(regionFileReader);
         PlanarRegion loadedRegion = loadPlanarRegionVertices(regionBufferedReader,
                                                              concaveHullSize.intValue(),
                                                              convexPolygonsSize.toArray(),
                                                              regionId.intValue(),
                                                              origin,
                                                              normal);
         regionBufferedReader.close();

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

   private static PlanarRegionsList importPlanarRegionDataFromFileInternal(File file) throws IOException
   {
      Scanner scan = new Scanner(file);
      List<PlanarRegion> planarRegions = new ArrayList<>();

      scan.useDelimiter("\\*\n");

      BufferedReader headerBufferedReader = new BufferedReader(new StringReader(scan.next()));

      HashMap<String, String> regionStrings = new HashMap<>();

      while (scan.hasNext())
      {
         scan.nextLine(); //Skip past delimiter
         regionStrings.put(scan.nextLine(), scan.next());
      }

      while (true)
      {
         Point3D origin = new Point3D();
         Vector3D normal = new Vector3D();
         MutableInt regionId = new MutableInt();
         MutableInt concaveHullSize = new MutableInt();
         TIntArrayList convexPolygonsSize = new TIntArrayList();

         String regionName = readHeaderLine(headerBufferedReader, origin, normal, regionId, concaveHullSize, convexPolygonsSize);

         if (regionName == null)
            break;

         if (!regionStrings.containsKey(regionName))
            continue;

         BufferedReader regionBufferedReader = new BufferedReader(new StringReader(regionStrings.get(regionName)));
         PlanarRegion loadedRegion = loadPlanarRegionVertices(regionBufferedReader,
                                                              concaveHullSize.intValue(),
                                                              convexPolygonsSize.toArray(),
                                                              regionId.intValue(),
                                                              origin,
                                                              normal);
         regionBufferedReader.close();

         if (loadedRegion != null)
            planarRegions.add(loadedRegion);
         else
         {
            headerBufferedReader.close();
            return null;
         }
      }

      headerBufferedReader.close();

      return new PlanarRegionsList(planarRegions);
   }

   private interface FileCreator
   {
      File createFile(String filename);
   }

   private static PlanarRegionsList importPlanarRegionDataInternalForTests(ReaderCreator readerCreator) throws IOException
   {
      BufferedReader headerFile = readerCreator.createReader("header.txt");
      if (headerFile == null)
      {
         LogTools.error("Could not find header file for planar region!");
         return null;
      }

      List<PlanarRegion> planarRegions = new ArrayList<>();

      while (true)
      {
         Point3D origin = new Point3D();
         Vector3D normal = new Vector3D();
         MutableInt regionId = new MutableInt();
         MutableInt concaveHullSize = new MutableInt();
         TIntArrayList convexPolygonsSize = new TIntArrayList();

         String fileName = readHeaderLine(headerFile, origin, normal, regionId, concaveHullSize, convexPolygonsSize);

         if (fileName == null)
            break;

         BufferedReader regionFile = readerCreator.createReader(fileName);
         if (regionFile == null)
            return null;

         PlanarRegion loadedRegion = loadPlanarRegionVertices(regionFile,
                                                              concaveHullSize.intValue(),
                                                              convexPolygonsSize.toArray(),
                                                              regionId.intValue(),
                                                              origin,
                                                              normal);
         regionFile.close();

         if (loadedRegion != null)
         {
            planarRegions.add(loadedRegion);
         }
         else
         {
            headerFile.close();
            return null;
         }
      }

      headerFile.close();

      return new PlanarRegionsList(planarRegions);
   }

   private interface ReaderCreator
   {
      BufferedReader createReader(String filename);
   }

   private static String readHeaderLine(BufferedReader bufferedReader,
                                        Point3D originToPack,
                                        Vector3D normalToPack,
                                        MutableInt regionIdToPack,
                                        MutableInt concaveHullSizeToPack,
                                        TIntArrayList convexPolygonsSizeToPack) throws IOException
   {
      String line = bufferedReader.readLine();
      if (line == null)
         return null;

      String cvsSplitBy = ",";

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

      regionIdToPack.setValue(Integer.parseInt(values[i++]));
      int regionIndex = Integer.parseInt(values[i++]);

      float xOrigin = Float.parseFloat(values[i++]);
      float yOrigin = Float.parseFloat(values[i++]);
      float zOrigin = Float.parseFloat(values[i++]);
      originToPack.set(xOrigin, yOrigin, zOrigin);

      float xNormal = Float.parseFloat(values[i++]);
      float yNormal = Float.parseFloat(values[i++]);
      float zNormal = Float.parseFloat(values[i++]);
      normalToPack.set(xNormal, yNormal, zNormal);

      concaveHullSizeToPack.setValue(Integer.parseInt(values[i++]));
      int numberOfConvexPolygons = Integer.parseInt(values[i++]);

      for (int hullIndex = 0; hullIndex < numberOfConvexPolygons; hullIndex++)
         convexPolygonsSizeToPack.add(Integer.parseInt(values[i++]));

      return "region" + regionIdToPack.toString() + "_" + regionIndex;
   }

   private static void writePlanarRegionsDataToStream(OutputStream ostream, PlanarRegionsList planarRegionData) throws IOException
   {
      OutputStreamWriter ow = new OutputStreamWriter(ostream);

      HashMap<String, Pair<PlanarRegion, Integer>> writeQueue = writePlanarRegionHeader(ow, planarRegionData);

      for (Map.Entry<String, Pair<PlanarRegion, Integer>> entry : writeQueue.entrySet())
      {
         int regionIndex = entry.getValue().getSecond();
         PlanarRegion region = entry.getValue().getFirst();

         ow.write("*\nregion" + entry.getKey() + "\n"); //Separate entries
         writePlanarRegionVertices(ow, region);
      }

      ow.flush();
   }

   private static void writePlanarRegionsData(Path folderPath, PlanarRegionsList planarRegionData) throws IOException
   {
      File header = new File(folderPath.toFile(), "header.txt");

      FileWriter headerWriter = new FileWriter(header);
      HashMap<String, Pair<PlanarRegion, Integer>> writeQueue = writePlanarRegionHeader(headerWriter, planarRegionData);
      headerWriter.close();

      for (Map.Entry<String, Pair<PlanarRegion, Integer>> entry : writeQueue.entrySet())
      {
         int regionIndex = entry.getValue().getSecond();
         PlanarRegion region = entry.getValue().getFirst();

         File regionFile = new File(folderPath.toFile(), "region" + entry.getKey());
         FileWriter fileWriter = new FileWriter(regionFile);
         writePlanarRegionVertices(fileWriter, region);
         fileWriter.close();
      }
   }

   private static HashMap<String, Pair<PlanarRegion, Integer>> writePlanarRegionHeader(OutputStreamWriter fw, PlanarRegionsList planarRegionData) throws IOException
   {
      HashMap<String, Pair<PlanarRegion, Integer>> writeQueue = new HashMap<>();
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

         fw.write("regionId: " + Integer.toString(regionId));
         fw.write(", index: " + Integer.toString(regionIndex.getValue().intValue()));
         fw.write(", origin: " + origin.getX() + ", " + origin.getY() + ", " + origin.getZ());
         fw.write(", normal: " + normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
         fw.write(", concave hull size: " + region.getConcaveHullSize());
         fw.write(", number of convex polygons: " + numberOfConvexPolygons + ", " + Arrays.toString(convexPolygonsSizes));

         fw.write("\n");

         writeQueue.put(region.getRegionId() + "_" + regionIndex, new Pair<PlanarRegion, Integer>(region, regionIndex.getValue()));
      }

      return writeQueue;
   }

   private static void writePlanarRegionVertices(OutputStreamWriter fileWriter, PlanarRegion region) throws IOException
   {
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
   }

   private static PlanarRegion loadPlanarRegionVertices(BufferedReader regionFile,
                                                        int concaveHullSize,
                                                        int[] convexPolygonsSize,
                                                        int regionId,
                                                        Point3D origin,
                                                        Vector3D normal)
   {
      if (regionFile == null)
         return null;

      try
      {
         String line = "";
         String cvsSplitBy = ",";

         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);
         RigidBodyTransform transformToWorld = new RigidBodyTransform(orientation, origin);

         List<Point2D> loadedPoints = new ArrayList<>();

         while ((line = regionFile.readLine()) != null)
         {
            String[] coordsAsString = line.split(cvsSplitBy);
            float x = Float.parseFloat(coordsAsString[0]);
            float y = Float.parseFloat(coordsAsString[1]);
            loadedPoints.add(new Point2D(x, y));
         }

         List<Point2D> concaveHullVertices = new ArrayList<>();

         for (int i = 0; i < concaveHullSize; i++)
         {
            concaveHullVertices.add(loadedPoints.remove(0));
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

   public static File fileFromClassPath(Class<?> loadingClass, Path path)
   {
      String builder = "";
      for (int i = 0; i < path.getNameCount(); i++)
      {
         builder += path.getName(i).toString();
         if (i < path.getNameCount() - 1)
         {
            builder += "/";
         }
      }

      URL resourceUrl = loadingClass.getClassLoader().getResource(builder);
      String resourcePath = resourceUrl.getPath();

      if (isWindows())
         resourcePath = resourcePath.substring(1, resourcePath.length());

      Path newPath = Paths.get(resourcePath);

      return newPath.toFile();
   }

   public static boolean isWindows()
   {
      String OS = System.getProperty("os.name").toLowerCase();
      return (OS.contains("win"));
   }

   public static List<String> listResourceDirectoryContents(Class<?> loadingClass, Path relativePath)
   {
      return listResourceDirectoryContents(loadingClass, relativePath.toString());
   }

   public static List<String> listResourceDirectoryContents(Class<?> loadingClass, String relativePath)
   {
      try
      {
         System.out.println("=================================================================");
         System.out.println(relativePath);
         return IOUtils.readLines(loadingClass.getClassLoader().getResourceAsStream(relativePath), StandardCharsets.UTF_8.name());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e.getMessage());
      }
   }

   public static URL getResourceURL(String resourceName)
   {
      return Thread.currentThread().getContextClassLoader().getResource(resourceName);
   }

   public static File getResourceFile(String resourceName)
   {
      URL url = getResourceURL(resourceName);
      return new File(url.getFile());
   }
}
