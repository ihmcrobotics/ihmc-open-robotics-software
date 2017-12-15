package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class VisibilityGraphsIOTools
{
   private static final String UNIT_TEST_FOLDER_SUFFIX = "_UnitTest";
   private static final String UNIT_TEST_PARAMETERS_FILENAME = "UnitTestParameters.txt";

   private static final String PATH_SIZE_FIELD_OPEN = "<PathSize,";
   private static final String PATH_SIZE_FIELD_END = ",PathSize>";

   private static final String START_FIELD_OPEN = "<Start,";
   private static final String START_FIELD_CLOSE = ",Start>";

   private static final String GOAL_FIELD_OPEN = "<Goal,";
   private static final String GOAL_FIELD_END = ",Goal>";

   public static boolean isWindows()
   {
      String OS = System.getProperty("os.name").toLowerCase();
      return (OS.contains("win"));
   }

   static Point3D parsePoint3D(String stringPoint3D)
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

   private static interface Parser<T>
   {
      T parse(String string);
   }

   public static File getDataResource(Class<?> loadingClass)
   {
      String pathString = loadingClass.getClassLoader().getResource("Data").getPath();
      if (isWindows())
         pathString = pathString.substring(1, pathString.length());

      return Paths.get(pathString).toFile();
   }

   public static List<VisibilityGraphsUnitTestDataset> loadAllDatasets(Class<?> loadingClass)
   {
      File mainDataFolder = getDataResource(loadingClass);

      return Arrays.stream(mainDataFolder.listFiles()).map(VisibilityGraphsIOTools::loadDataset).filter(dataset -> dataset != null)
                   .collect(Collectors.toList());
   }

   public static VisibilityGraphsUnitTestDataset loadDataset(File datasetFile)
   {
      if (!isFileUnitTestDatasetFolder(datasetFile))
         return null;
      else
         return new VisibilityGraphsUnitTestDataset(datasetFile);
   }

   private static boolean isFileUnitTestDatasetFolder(File fileFolder)
   {
      if (!fileFolder.isDirectory())
         return false;

      if (!fileFolder.getName().endsWith(UNIT_TEST_FOLDER_SUFFIX))
         return false;

      return true;
   }

   public static class VisibilityGraphsUnitTestDataset
   {
      private final File file;

      private final int expectedPathSize;
      private final Point3D start;
      private final Point3D goal;
      private final PlanarRegionsList planarRegionsList;

      private VisibilityGraphsUnitTestDataset(File file)
      {
         this.file = file;
         String simpleFileName = file.getName().replace(UNIT_TEST_FOLDER_SUFFIX, "");
         File planarRegionFile = new File(file.getPath(), simpleFileName);
         File parameterFile = new File(file.getPath(), UNIT_TEST_PARAMETERS_FILENAME);

         expectedPathSize = parsePathSize(parameterFile);
         start = parseField(parameterFile, START_FIELD_OPEN, START_FIELD_CLOSE, VisibilityGraphsIOTools::parsePoint3D);
         goal = parseField(parameterFile, GOAL_FIELD_OPEN, GOAL_FIELD_END, VisibilityGraphsIOTools::parsePoint3D);
         planarRegionsList = PlanarRegionDataImporter.importPlanRegionData(planarRegionFile);

         if (start == null)
            throw new RuntimeException("Could not load the start position. Data file: " + parameterFile);
         if (goal == null)
            throw new RuntimeException("Could not load the goal position. Data file: " + parameterFile);
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
         return file.getName();
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
