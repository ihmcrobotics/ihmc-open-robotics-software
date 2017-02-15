package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.gui.MyFileFilter;

public class ImportDataDialogGenerator implements ImportDataDialogConstructor
{
   private JFileChooser fileChooser;
   private File chosenFile;

   private javax.swing.filechooser.FileFilter stateFileFilter = new MyFileFilter(new String[] {".state", ".state.gz"}, "State (.state, .state.gz)");
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".data", ".data.gz", ".data.csv"},
                                                                  "Data (.data, .data.gz, .data.csv)");
   private javax.swing.filechooser.FileFilter stateOrDataFileFilter = new MyFileFilter(new String[] {".data", ".data.gz", ".data.csv", ".state", ".state.gz"},
                                                                         "Data or State (.data, .data.gz, .data.csv, .state, .state.gz)");

   // private javax.swing.filechooser.FileFilter jpegFileFilter = new MyFileFilter(new String[]{".jpg", ".jpeg"}, "JPEG (.jpg, .jpeg)");
   // private javax.swing.filechooser.FileFilter configFileFilter = new MyFileFilter(".config", "Configuration (.config)");


   private SimulationConstructionSet sim;
   private JFrame frame;

   public ImportDataDialogGenerator(SimulationConstructionSet sim, Robot[] robots, JFrame frame)
   {
      this.sim = sim;
      this.frame = frame;

      try
      {
         fileChooser = new JFileChooser();

         if (robots != null)
         {
            URL defaultDirURL = robots.getClass().getResource(".");
            if (defaultDirURL != null)
            {
               String defaultDirString = defaultDirURL.getPath();
               if (defaultDirString != null)
               {
                  int idx = defaultDirString.indexOf("classes");
                  if (idx > 0)
                     defaultDirString = defaultDirString.substring(0, idx);

                  setCurrentDirectory(defaultDirString);
               }
            }
         }

         // fileChooser.setApproveButtonText("Save ME");
         fileChooser.setAcceptAllFileFilterUsed(true);
         fileChooser.addChoosableFileFilter(stateFileFilter);
         fileChooser.addChoosableFileFilter(dataFileFilter);
         fileChooser.addChoosableFileFilter(this.stateOrDataFileFilter);
      }
      catch (Exception e)
      {
      }
   }

   @Override
   public void setCurrentDirectory(File dir)
   {
      fileChooser.setCurrentDirectory(dir);
   }

   @Override
   public void setCurrentDirectory(String dir)
   {
      fileChooser.setCurrentDirectory(new File(dir));
   }


   @Override
   public void constructDialog()
   {
      sim.disableGUIComponents();

      if (fileChooser.showOpenDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         chosenFile = fileChooser.getSelectedFile();

         if (chosenFile.canRead()
                 && (chosenFile.getName().endsWith(".data") || chosenFile.getName().endsWith(".data.gz") || chosenFile.getName().endsWith(".data.csv")))
         {
            sim.readData(chosenFile);
         }

         else if (chosenFile.canRead() && (chosenFile.getName().endsWith(".state") || chosenFile.getName().endsWith(".state.gz")))
         {
            sim.readState(chosenFile);
         }

         else
         {
            JOptionPane.showMessageDialog(frame, "File not found or not readable!");
         }
      }

      sim.enableGUIComponents();
   }

   @Override
   public void closeAndDispose()
   {
      fileChooser = null;
      chosenFile = null;

      stateFileFilter = null;
      dataFileFilter  = null;
      stateOrDataFileFilter = null;

      sim = null;
      frame = null;
   }
}

