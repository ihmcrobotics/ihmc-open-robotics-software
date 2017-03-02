package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;
import java.net.URL;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.ExportDataDialogListener;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.gui.dialogs.ExportDataDialog;
import us.ihmc.simulationconstructionset.gui.dialogs.SCSExportDataFormat;
import us.ihmc.tools.gui.MyFileFilter;

public class ExportDataDialogGenerator implements ExportDataDialogConstructor, ExportDataDialogListener
{
   private JFrame frame;
   private SimulationConstructionSet sim;
   private VarGroupList varGroupList;

   private JFileChooser dataFileChooser, stateFileChooser;
   private File chosenFile;

   private javax.swing.filechooser.FileFilter stateFileFilter = new MyFileFilter(new String[] {".state", ".state.gz"}, "State (.state, .state.gz)");
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".data", ".data.gz", ".data.csv", ".mat"},
                                                                  "Data (.data, .data.gz, .data.csv, .mat)");


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

   @Override
   public void closeAndDispose()
   {
      frame = null;
      sim = null;
      varGroupList = null;

      dataFileChooser = null;
      stateFileChooser = null;
      chosenFile = null;

      stateFileFilter = null;
      dataFileFilter = null;
   }

   @Override
   public void setCurrentDirectory(File dir)
   {
      stateFileChooser.setCurrentDirectory(dir);
      dataFileChooser.setCurrentDirectory(dir);
   }

   @Override
   public void setCurrentDirectory(String dir)
   {
      dataFileChooser.setCurrentDirectory(new File(dir));
      stateFileChooser.setCurrentDirectory(new File(dir));
   }


   @Override
   public void constructDialog()
   {
      new ExportDataDialog(frame, varGroupList, this);
   }

   @Override
   public void export(String varGroup, int dataType, SCSExportDataFormat dataFormat, int dataCompression)
   {
      // if (true) return;
      sim.disableGUIComponents();

      // Determine what the file ending should be...
      String fileEnding;
      if (dataType == ExportDataDialog.DATA)
         fileEnding = ".data";
      else
         fileEnding = ".state";
      
      if ((dataFormat==SCSExportDataFormat.ASCII || dataFormat==SCSExportDataFormat.BINARY) &&dataCompression == ExportDataDialog.COMPRESS)
         fileEnding = fileEnding.concat(".gz");

      if (dataFormat == SCSExportDataFormat.SPREADSHEET)
         fileEnding = fileEnding.concat(".csv");
      else if (dataFormat == SCSExportDataFormat.MATLAB)
         fileEnding = ".mat"; //matlab hates .data.mat so replace instead of concat


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

            boolean compress = (dataCompression == ExportDataDialog.COMPRESS);

            switch (dataFormat)
            {
               case ASCII :
                  sim.writeData(varGroup, false, compress, chosenFile);
                  break;

               case BINARY :
                  sim.writeData(varGroup, true, compress, chosenFile);
                  break;

               case MATLAB :
                  sim.writeMatlabData(varGroup, chosenFile);
                  break;

               case SPREADSHEET :
                  sim.writeSpreadsheetFormattedData(varGroup, chosenFile);
                  break;
            }
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

            boolean compress = (dataCompression == ExportDataDialog.COMPRESS);

            switch(dataFormat)
            {
            case ASCII:
               sim.writeState(varGroup, false, compress, chosenFile);
               break;
            case BINARY:
               sim.writeState(varGroup, true, compress, chosenFile);
               break;
            case SPREADSHEET:
               sim.writeSpreadsheetFormattedState(varGroup, chosenFile);
               break;
            case MATLAB:
               sim.writeMatlabData(varGroup, chosenFile);
            }
         }
      }

      sim.enableGUIComponents();
   }
}
