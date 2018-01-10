package us.ihmc.pathPlanning.visibilityGraphs.tools;

import org.apache.commons.io.IOUtils;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.*;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class VisibilityGraphsIOTools
{
   public static final String DATA_FOLDER_NAME = "us/ihmc/pathPlanning/visibilityGraphs/data";
   private static final String VIZ_GRAPHS_DATA_FOLDER_SUFFIX = "VizGraphs";
   public static final String INPUTS_PARAMETERS_FILENAME = "VizGraphsInputs.txt";

   private static final String PATH_SIZE_FIELD_OPEN = "<PathSize,";
   private static final String PATH_SIZE_FIELD_END = ",PathSize>";

   private static final String START_FIELD_OPEN = "<Start,";
   private static final String START_FIELD_CLOSE = ",Start>";

   private static final String GOAL_FIELD_OPEN = "<Goal,";
   private static final String GOAL_FIELD_END = ",Goal>";

   public static boolean exportDataset(Path containingFolder, String datasetName, PlanarRegionsList planarRegionsList, Point3DReadOnly start,
                                       Point3DReadOnly goal)
   {
      File datasetFolder = new File(containingFolder + File.separator + datasetName);
      if (datasetFolder.exists())
         return false;
      boolean success = datasetFolder.mkdir();
      if (!success)
         return false;

      Path planarRegionsFolder = Paths.get(datasetFolder.getPath() + File.separator + PlanarRegionFileTools.createDefaultTimeStampedFolderName());
      success = PlanarRegionFileTools.exportPlanarRegionData(planarRegionsFolder, planarRegionsList);
      if (!success)
         return false;

      success = exportParameters(datasetFolder, start, goal);
      if (!success)
         return false;

      return true;
   }

   private static boolean exportParameters(File containingFolder, Point3DReadOnly start, Point3DReadOnly goal)
   {
      if (containingFolder == null || !containingFolder.exists())
      {
         PrintTools.error("The given folder does not exist or is null.");
         return false;
      }

      if (start == null || goal == null)
      {
         PrintTools.error("Must export start AND goal.");
         return false;
      }

      File parametersFile = new File(containingFolder.getAbsolutePath() + File.separator + INPUTS_PARAMETERS_FILENAME);
      writeField(parametersFile, START_FIELD_OPEN, START_FIELD_CLOSE, () -> getPoint3DString(start));
      writeField(parametersFile, GOAL_FIELD_OPEN, GOAL_FIELD_END, () -> getPoint3DString(goal));

      return true;
   }

   /**
    * Generates a default timestamped name that can be used to generate automated and unique
    * folders.
    *
    * @return a {@code String} of the form: "20171201_163422_VizGraphs".
    */
   public static String createDefaultTimeStampedDatasetFolderName()
   {
      return PlanarRegionFileTools.getDate() + "_" + VIZ_GRAPHS_DATA_FOLDER_SUFFIX;
   }

   public static boolean isWindows()
   {
      String OS = System.getProperty("os.name").toLowerCase();
      return (OS.contains("win"));
   }

   private static String getPoint3DString(Point3DReadOnly point3D)
   {
      return EuclidCoreIOTools.getStringOf("", "", ",", point3D.getX(), point3D.getY(), point3D.getZ());
   }

   private static Point3D parsePoint3D(String stringPoint3D)
   {
      double x = Double.parseDouble(stringPoint3D.substring(0, stringPoint3D.indexOf(",")));
      stringPoint3D = stringPoint3D.substring(stringPoint3D.indexOf(",") + 1);
      double y = Double.parseDouble(stringPoint3D.substring(0, stringPoint3D.indexOf(",")));
      stringPoint3D = stringPoint3D.substring(stringPoint3D.indexOf(",") + 1);
      double z = Double.parseDouble(stringPoint3D.substring(0));

      return new Point3D(x, y, z);
   }

   private static <T> T parseField(File file, String fieldOpen, String fieldClose, Parser<T> parser)
   {
      BufferedReader br = null;
      FileReader fr = null;

      try
      {
         fr = new FileReader(file);
         br = new BufferedReader(fr);

         String sCurrentLine;

         while ((sCurrentLine = br.readLine()) != null)
         {
            if (sCurrentLine.contains(fieldOpen) && sCurrentLine.contains(fieldClose))
            {
               return parser.parse(sCurrentLine.substring(fieldOpen.length(), sCurrentLine.indexOf(fieldClose)));
            }
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();

      } finally
      {
         try
         {
            if (br != null)
               br.close();

            if (fr != null)
               fr.close();
         }
         catch (IOException ex)
         {
            ex.printStackTrace();
         }
      }
      return null;
   }

   private static void writeField(File file, String fieldOpen, String fieldClose, Writer writer)
   {
      BufferedWriter bw = null;

      try
      {
         if (!file.exists())
            file.createNewFile();

         FileWriter fw = new FileWriter(file.getAbsoluteFile(), true);
         bw = new BufferedWriter(fw);

         bw.write(fieldOpen + writer.getStringToWrite() + fieldClose);
         bw.newLine();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      } finally
      {
         try
         {
            if (bw != null)
               bw.close();
         }
         catch (IOException ex)
         {
            ex.printStackTrace();
         }
      }
   }

   private static interface Parser<T>
   {
      T parse(String string);
   }

   private static interface Writer
   {
      String getStringToWrite();
   }

   //   public static File getDataResource(Class<?> loadingClass)
   //   {
   //      URL url = loadingClass.getClassLoader().getResource(DATA_FOLDER_NAME);
   //      //      PrintTools.info(VisibilityGraphsIOTools.class, "url: " + url);
   //      String pathString = url.getPath();
   //      PrintTools.info(VisibilityGraphsIOTools.class, "pathString bfore: " + pathString);
   //      if (isWindows())
   //         pathString = pathString.substring(1, pathString.length());
   //
   //      //      PrintTools.info(VisibilityGraphsIOTools.class, "pathString: " + pathString);
   //
   //      Path path = Paths.get(pathString);
   //
   //      //      loadingClass.getClassLoader().
   //      //      PrintTools.info(VisibilityGraphsIOTools.class, "path: " + path.toString());
   //      return path.toFile();
   //   }

   public static List<VisibilityGraphsUnitTestDataset> loadAllDatasets(Class<?> loadingClass)
   {
      //      File mainDataFolder = getDataResource(loadingClass);

      //      PrintTools.info(VisibilityGraphsIOTools.class, "mainDataFolder: " + mainDataFolder.getPath());

      ArrayList<Path> relativePaths = new ArrayList<>();
      listResourceDirectoryContents(loadingClass, DATA_FOLDER_NAME).forEach(entry -> relativePaths.add(Paths.get(DATA_FOLDER_NAME, entry)));

      return relativePaths.stream().map(VisibilityGraphsIOTools::loadDataset).collect(Collectors.toList());
   }

   private static List<String> listResourceDirectoryContents(Class<?> loadingClass, String relativePath)
   {
      try
      {
         return IOUtils.readLines(loadingClass.getClassLoader().getResourceAsStream(relativePath), StandardCharsets.UTF_8);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e.getMessage());
      }
   }

   public static File fileFromClassPath(Path path)
   {
      String pathString = path.toString();

      String builder = "";
      for (int i = 0; i < path.getNameCount(); i++)
      {
         builder += path.getName(i).toString();
         if (i < path.getNameCount() - 1)
         {
            builder += "/";
         }
      }

      PrintTools.info(VisibilityGraphsIOTools.class, "Going in: " + builder);

      URL resourceUrl = VisibilityGraphsIOTools.class.getClassLoader().getResource(builder);
      PrintTools.info(VisibilityGraphsIOTools.class, "resourceUrl: " + resourceUrl.toString());
      String resourcePath = resourceUrl.getPath();
      PrintTools.info(VisibilityGraphsIOTools.class, "resourcePath: " + resourcePath);
      if (isWindows())
         resourcePath = resourcePath.substring(1, resourcePath.length());

      Path newPath = Paths.get(resourcePath);
      PrintTools.info(VisibilityGraphsIOTools.class, "newPath: " + newPath);

      return newPath.toFile();
   }

   /**
    * Loads either directly the planar regions from the given file, or from the first child
    * directory if the file is a visibility graphs dataset.
    *
    * @param inputFile the file to load the planar regions from. Can be an actual planar regions
    *           data file or a visibility graphs dataset file.
    * @return the loaded planar regions or {@code null} if not able to load them.
    */
   public static PlanarRegionsList importPlanarRegionData(File inputFile)
   {
      return PlanarRegionFileTools.importPlanRegionData(adjustPlanarRegionsDataFile(inputFile));
   }

   /**
    * Adjust the file path such that the given file is returned if it is a planar regions file, or
    * the first child directory is returned if it is a visibility graphs dataset file.
    *
    * @param inputFile the planar regions file to be adjusted. Can be an actual planar regions data
    *           file or a visibility graphs dataset file.
    * @return
    */
   private static File adjustPlanarRegionsDataFile(File inputFile)
   {
      File[] vizGraphsParameterFile = inputFile.listFiles((FilenameFilter) (dir, name) -> name.equals(VisibilityGraphsIOTools.INPUTS_PARAMETERS_FILENAME));
      if (vizGraphsParameterFile != null && vizGraphsParameterFile.length == 1)
         inputFile = inputFile.listFiles(File::isDirectory)[0];
      return inputFile;
   }

   public static String[] getPlanarRegionAndVizGraphsFilenames(File parentFolder)
   {
      if (!parentFolder.exists() || !parentFolder.isDirectory())
         return null;

      return Arrays.stream(parentFolder.listFiles(file -> isVisibilityGraphsDataset(file) || PlanarRegionFileTools.isPlanarRegionFile(file))).map(File::getName)
                   .toArray(String[]::new);
   }

   public static String[] getSubDirectoriesNames(File parentFolder)
   {
      if (!parentFolder.exists() || !parentFolder.isDirectory())
         return null;

      return Arrays.stream(parentFolder.listFiles(File::isDirectory)).map(File::getName).toArray(String[]::new);
   }

   public static boolean isVisibilityGraphsDataset(File dataFolder)
   {
      if (dataFolder == null || !dataFolder.exists() || !dataFolder.isDirectory())
         return false;

      File[] paramsFiles = dataFolder.listFiles((dir, name) -> name.equals(INPUTS_PARAMETERS_FILENAME));

      if (paramsFiles == null || paramsFiles.length != 1)
         return false;

      File[] planarRegionFolders = dataFolder.listFiles(File::isDirectory);

      if (planarRegionFolders == null || planarRegionFolders.length != 1)
         return false;

      return PlanarRegionFileTools.isPlanarRegionFile(planarRegionFolders[0]);
   }

   public static VisibilityGraphsUnitTestDataset loadDataset(Path datasetFolder)
   {
      //      if (!datasetFolder.isDirectory())
      //      {
      //         throw new RuntimeException("The data folder should only contain valid datasets for visibility graphs. Bad file: " + datasetFolder);
      //      }

      return new VisibilityGraphsUnitTestDataset(datasetFolder);
   }

   public static class VisibilityGraphsUnitTestDataset
   {
      private final Path datasetPath;

      private final int expectedPathSize;
      private final Point3D start;
      private final Point3D goal;
      private final PlanarRegionsList planarRegionsList;

      private VisibilityGraphsUnitTestDataset(Path datasetPath)
      {
         this.datasetPath = datasetPath;
         PrintTools.info("Dataset path: " + datasetPath);

         Path parametersPath = datasetPath.resolve(INPUTS_PARAMETERS_FILENAME);

         PrintTools.info("expectedParametersFileName: " + parametersPath);

         //         List<File> children = Arrays.asList(file.listFiles());
         //         File parametersFile = children.stream().filter(child -> child.getName().endsWith(INPUTS_PARAMETERS_FILENAME)).findFirst().orElse(null);

         if (getClass().getClassLoader().getResource(parametersPath.toString()) == null)
            throw new RuntimeException("Could not find the parmeter file: " + parametersPath);

         String planarRegionDirectoryName = listResourceDirectoryContents(getClass(), datasetPath.toString()).stream().filter(
               content -> content.contains("PlanarRegion")).findFirst().orElse(null);
         PrintTools.info(this, "PlanarRegion folder: " + planarRegionDirectoryName);

         //         File planarRegionFile = children.stream().filter(child -> child.isDirectory()).findFirst().orElse(null);

         if (planarRegionDirectoryName == null)
            throw new RuntimeException("Could not find the planar region directory.");

         File parametersFile = fileFromClassPath(parametersPath);
         File planarRegionFile = fileFromClassPath(datasetPath.resolve(planarRegionDirectoryName));

         expectedPathSize = parsePathSize(parametersFile);
         start = parseField(parametersFile, START_FIELD_OPEN, START_FIELD_CLOSE, VisibilityGraphsIOTools::parsePoint3D);
         goal = parseField(parametersFile, GOAL_FIELD_OPEN, GOAL_FIELD_END, VisibilityGraphsIOTools::parsePoint3D);
         planarRegionsList = PlanarRegionDataImporter.importPlanRegionData(planarRegionFile);

         if (start == null)
            throw new RuntimeException("Could not load the start position. Data file: " + parametersFile);
         if (goal == null)
            throw new RuntimeException("Could not load the goal position. Data file: " + parametersFile);
         if (planarRegionsList == null)
            throw new RuntimeException("Could not load the planar regions. Data file: " + planarRegionFile);
      }

      private static int parsePathSize(File file)
      {
         Integer pathSize = parseField(file, PATH_SIZE_FIELD_OPEN, PATH_SIZE_FIELD_END, Integer::valueOf);
         if (pathSize == null)
            return -1;
         else
            return pathSize.intValue();
      }

      public String getDatasetName()
      {
         return datasetPath.toString();
      }

      public Point3D getStart()
      {
         return new Point3D(start);
      }

      public Point3D getGoal()
      {
         return new Point3D(goal);
      }

      public PlanarRegionsList getPlanarRegionsList()
      {
         return planarRegionsList;
      }

      public boolean hasExpectedPathSize()
      {
         return expectedPathSize > 0;
      }

      public int getExpectedPathSize()
      {
         return expectedPathSize;
      }
   }
}
