package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.ExportDataDialogListener;
import us.ihmc.simulationconstructionset.gui.MyFileFilter;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.gui.dialogs.ExportDataDialog;

public class ExportDataDialogGenerator implements ExportDataDialogConstructor, ExportDataDialogListener
{
   private JFrame frame;
   private SimulationConstructionSet sim;
   private VarGroupList varGroupList;

   private JFileChooser dataFileChooser, stateFileChooser;
   private File chosenFile;

   private javax.swing.filechooser.FileFilter stateFileFilter = new MyFileFilter(new String[] {".state", ".state.gz"}, "State (.state, .state.gz)");
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".data", ".data.gz", ".data.csv"},
                                                                  "Data (.data, .data.gz, .data.csv)");


   public ExportDataDialogGenerator(SimulationConstructionSet sim, VarGroupList varGroupList, JFrame frame)
   {
      this.frame = frame;
      this.sim = sim;
      this.varGroupList = varGroupList;

      try    // +++++++++JEP Applet Stuff
      {
         stateFileChooser = new JFileChooser();
         dataFileChooser = new JFileChooser();

         if (sim != null)
         {
            URL defaultDirURL = sim.getClass().getResource(".");
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
         dataFileChooser.setAcceptAllFileFilterUsed(false);
         stateFileChooser.setAcceptAllFileFilterUsed(false);
         stateFileChooser.addChoosableFileFilter(stateFileFilter);
         dataFileChooser.addChoosableFileFilter(dataFileFilter);
      }
      catch (Exception e)
      {
         // e.printStackTrace();
      }
   }

   public void closeAndDispose()
   {
      frame = null;
      sim = null;
      varGroupList = null;

      dataFileChooser = null; stateFileChooser = null;
      chosenFile = null;

      stateFileFilter = null;
      dataFileFilter = null;
   }
   
   public void setCurrentDirectory(File dir)
   {
      stateFileChooser.setCurrentDirectory(dir);
      dataFileChooser.setCurrentDirectory(dir);
   }

   public void setCurrentDirectory(String dir)
   {
      dataFileChooser.setCurrentDirectory(new File(dir));
      stateFileChooser.setCurrentDirectory(new File(dir));
   }


   public void constructExportDataDialog()
   {
      new ExportDataDialog(frame, varGroupList, this);
   }

   public void export(String varGroup, int dataType, int dataFormat, int dataCompression, boolean spreadsheetFormatted)
   {
      // if (true) return;
      sim.disableGUIComponents();

      // Determine what the file ending should be...
      String fileEnding;
      if (dataType == ExportDataDialog.DATA)
         fileEnding = ".data";
      else
         fileEnding = ".state";
      if (dataCompression == ExportDataDialog.COMPRESS)
         fileEnding = fileEnding.concat(".gz");

      if ((spreadsheetFormatted) && (dataType == ExportDataDialog.DATA) && (dataCompression == ExportDataDialog.NO_COMPRESS))
         fileEnding = fileEnding.concat(".csv");


      if (dataType == ExportDataDialog.DATA)
      {
         if (dataFileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
         {
            chosenFile = dataFileChooser.getSelectedFile();
            String filename = chosenFile.getName();

            if (!filename.endsWith(fileEnding))
            {
               filename = filename.concat(fileEnding);

               if (!chosenFile.getName().equals(filename))
               {
                  File newChosenFile = new File(chosenFile.getParent(), filename);
                  chosenFile = newChosenFile;
               }
            }

            boolean binary = (dataFormat == ExportDataDialog.BINARY);
            boolean compress = (dataCompression == ExportDataDialog.COMPRESS);

            if (!binary &&!compress && spreadsheetFormatted)
               sim.writeSpreadsheetFormattedData(varGroup, chosenFile);
            else
               sim.writeData(varGroup, binary, compress, chosenFile);
         }
      }

      else
      {
         if (stateFileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
         {
            chosenFile = stateFileChooser.getSelectedFile();
            String filename = chosenFile.getName();

            if (!filename.endsWith(fileEnding))
            {
               filename = filename.concat(fileEnding);

               if (!chosenFile.getName().equals(filename))
               {
                  File newChosenFile = new File(chosenFile.getParent(), filename);
                  chosenFile = newChosenFile;
               }
            }

            boolean binary = (dataFormat == ExportDataDialog.BINARY);
            boolean compress = (dataCompression == ExportDataDialog.COMPRESS);

            if (spreadsheetFormatted)
               sim.writeSpreadsheetFormattedState(varGroup, chosenFile);
            else
               sim.writeState(varGroup, binary, compress, chosenFile);
         }
      }

      sim.enableGUIComponents();
   }
}

