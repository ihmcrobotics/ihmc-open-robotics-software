package us.ihmc.pathPlanning.visibilityGraphs;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import us.ihmc.euclid.tuple3D.Point3D;

public class VisibilityGraphsIOTools
{
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

   public static void readStartGoalParameters(File file, Point3D startToPack, Point3D goalToPack)
   {
      startToPack.set(parseField(file, START_FIELD_OPEN, START_FIELD_CLOSE, VisibilityGraphsIOTools::parsePoint3D));
      goalToPack.set(parseField(file, GOAL_FIELD_OPEN, GOAL_FIELD_END, VisibilityGraphsIOTools::parsePoint3D));
   }

   public static int parsePathSize(File file)
   {
      Integer pathSize = parseField(file, PATH_SIZE_FIELD_OPEN, PATH_SIZE_FIELD_END, Integer::valueOf);
      if (pathSize == null)
         return -1;
      else
         return pathSize.intValue();
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
               return parser.parse(sCurrentLine.substring(10, sCurrentLine.indexOf(fieldClose)));
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
}
