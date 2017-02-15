package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.gui.MyFileFilter;

public class SaveRobotConfigurationDialogGenerator implements SaveRobotConfigurationDialogConstructor
{
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".robotConf"}, "Robot Configuration (.robotConf)");
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private SimulationConstructionSet sim;


   public SaveRobotConfigurationDialogGenerator(SimulationConstructionSet sim, JFrame frame)
   {
      this.frame = frame;
      this.sim = sim;

      try
      {
         dataFileChooser = new JFileChooser();

         if (sim != null)
         {
            URL defaultDirURL = sim.getClass().getResource(".");

            if (defaultDirURL != null)
            {
               String defaultDirString = defaultDirURL.getPath();

               if (defaultDirString != null)
               {
                  int index = defaultDirString.indexOf("classes");

                  if (index > 0)
                  {
                     defaultDirString = defaultDirString.substring(0, index);
                  }

                  setCurrentDirectory(defaultDirString);
               }
            }
         }

         // fileChooser.setApproveButtonText("Save ME");
         dataFileChooser.setAcceptAllFileFilterUsed(false);
         dataFileChooser.addChoosableFileFilter(dataFileFilter);
      }
      catch (Exception e)
      {
         // e.printStackTrace();
      }
   }

   @Override
   public void setCurrentDirectory(File directory)
   {
      dataFileChooser.setCurrentDirectory(directory);
   }

   @Override
   public void setCurrentDirectory(String directory)
   {
      dataFileChooser.setCurrentDirectory(new File(directory));
   }

   @Override
   public void constructDialog()
   {
      if (dataFileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         Robot robot = sim.getRobots()[0];
         File file = dataFileChooser.getSelectedFile();

         String fileEnding = ".robotConf";

         String filename = file.getName();

         if (!filename.endsWith(fileEnding))
         {
            filename = filename.concat(fileEnding);

            if (!file.getName().equals(filename))
            {
               File newChosenFile = new File(file.getParent(), filename);

               file = newChosenFile;
            }
         }

         if ((robot != null) && (file != null))
         {
            sim.exportRobotDefinition(robot, file);
         }
         else
         {
            System.err.println("File could not be written.");
         }
      }

   }

   public void closeAndDispose()
   {
      dataFileFilter = null;
      dataFileChooser = null;
      frame = null;
      sim = null;
   }

}
