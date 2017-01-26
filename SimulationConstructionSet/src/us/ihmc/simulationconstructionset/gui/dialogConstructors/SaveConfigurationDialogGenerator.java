package us.ihmc.simulationconstructionset.gui.dialogConstructors;


import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.Writer;

import javax.swing.JFileChooser;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.tools.gui.MyFileFilter;

public class SaveConfigurationDialogGenerator implements SaveConfigurationDialogConstructor
{
   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {".guiConf"}, "Gui Configuration (.guiConf)");
   private File chosenFile;
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private StandardSimulationGUI myGUI;
   private SimulationConstructionSet sim;


   public SaveConfigurationDialogGenerator(SimulationConstructionSet sim, JFrame frame, StandardSimulationGUI myGUI)
   {
      this.frame = frame;
      this.sim = sim;
      this.myGUI = myGUI;

      try
      {
         File Configs = new File("Configurations");

         String path = Configs.toURI().getPath();
         dataFileChooser = new JFileChooser();

         setCurrentDirectory(path);

         dataFileChooser.setAcceptAllFileFilterUsed(false);
         dataFileChooser.addChoosableFileFilter(dataFileFilter);
      }
      catch (Exception e)
      {
         // e.printStackTrace();
      }
   }

   @Override
   public void setCurrentDirectory(File dir)
   {
      dataFileChooser.setCurrentDirectory(dir);
   }

   @Override
   public void setCurrentDirectory(String dir)
   {
      dataFileChooser.setCurrentDirectory(new File(dir));
   }

   @Override
   public void constructDialog()
   {
      sim.disableGUIComponents();

      String fileEnding = ".guiConf";
      if (dataFileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         try
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

            Writer output = null;

            output = new BufferedWriter(new FileWriter(chosenFile));
            output.write(writeGUIConfig());

            output.close();
            System.out.println("Your file has been written");
         }
         catch (Exception e)
         {
            System.err.println("Error While Writting/Saving Gui Configuration File");
            e.printStackTrace();
         }
      }

      sim.enableGUIComponents();
   }

   private String writeGUIConfig()
   {
      String textToWrite = myGUI.getXMLStyleRepresentationOfGraphArrayPanel();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationOfEntryBoxes();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationOfViewPorts();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationOfGraphWindows();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationofJPanels();
      textToWrite += "\n" + myGUI.getXMLStyleRepresentationofMultiViews();

      return textToWrite;
   }

   public void closeAndDispose()
   {
      dataFileFilter = null;
      chosenFile = null;
      dataFileChooser = null;
      frame = null;
      myGUI = null;
      sim = null;
   }

}

