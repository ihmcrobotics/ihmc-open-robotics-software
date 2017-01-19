package us.ihmc.simulationconstructionset.dataExporter;

import java.awt.Component;
import java.io.File;
import java.net.URL;

import javax.swing.JOptionPane;

public class DataExporterDirectoryFinder
{
   public static File findSimulationRootLocation(Class<?> rootClassForDirectory)
   {
      Component component = null;
      if (rootClassForDirectory != null)
      {
         URL robotClassURL = rootClassForDirectory.getResource(".");
         if (robotClassURL == null)
         {
            JOptionPane.showMessageDialog(null, "Your robot class path URL is null", "Error Robot URL", JOptionPane.ERROR_MESSAGE);

            return null;
         }
         else
         {
            String robotClassDirectory = robotClassURL.getPath();
            if (robotClassDirectory == null)
            {
               JOptionPane.showMessageDialog(component, "Your path to the Robot class is null", "Error Robot URL", JOptionPane.ERROR_MESSAGE);

               return null;
            }
            else
            {
               String[] outputDirLocationsToCheck = new String[] {"classes", "bin"};

               String outputDir = null;
               for (String outputDirLocationToCheck : outputDirLocationsToCheck)
               {
                  int outputDirIndex = robotClassDirectory.indexOf(outputDirLocationToCheck);
                  if (outputDirIndex != -1)
                  {
                     outputDir = robotClassDirectory.substring(0, outputDirIndex);

                     break;
                  }
               }

               if (outputDir == null)
               {
                  StringBuilder builder = new StringBuilder();
                  builder.append("Your simulation output folder was not found. Accepted output folder names: \n");

                  for (String outputDirLocation : outputDirLocationsToCheck)
                  {
                     builder.append(outputDirLocation + "\n");
                  }

                  builder.append("Robot class directory: " + robotClassDirectory);

                  JOptionPane.showMessageDialog(component, builder.toString(), "Error finding simulation root location", JOptionPane.ERROR_MESSAGE);

                  return null;
               }

               File ret = new File(outputDir);

               if (!ret.exists())
               {
                  JOptionPane.showMessageDialog(component, "Could not find your simulation root location:\n" + outputDir,
                                                "Error finding simulation root location", JOptionPane.ERROR_MESSAGE);

                  return null;
               }
               else
               {
                  return ret;
               }
            }
         }
      }

      return null;
   }

   public static File findSimulationDataAndVideoRootLocation(File simulationRootDirectory, String subdirectoryName)
   {
      File ret = new File(simulationRootDirectory.getParentFile(), "DataAndVideos" + File.separator + subdirectoryName);
      if (!ret.exists())
         ret.mkdir();
      return ret;
   }
}
