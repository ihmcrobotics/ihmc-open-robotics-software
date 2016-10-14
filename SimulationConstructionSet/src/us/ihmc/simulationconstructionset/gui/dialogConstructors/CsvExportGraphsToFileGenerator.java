package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.BorderLayout;
import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;

import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.dataBuffer.DataEntry;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.MyFileFilter;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;
import us.ihmc.tools.io.files.FileTools;

public class CsvExportGraphsToFileGenerator implements ExportGraphsToFileConstructor
{
   private final static String fileEnding = ".csv";

   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {fileEnding}, "CSV file (.csv)");
   private File chosenFile;
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   private GraphArrayPanel graphArrayPanel;
   private StandardSimulationGUI myGUI;

   private JPanel accessory;
   private JCheckBox saveAllGraphs;
   
   private DataBuffer dataBuffer;

   public CsvExportGraphsToFileGenerator(SimulationConstructionSet scs, JFrame frame, GraphArrayPanel graphArrayPanel, StandardSimulationGUI myGUI)
   {
      this.frame = frame;
      this.guiEnablerAndDisabler = scs;
      this.graphArrayPanel = graphArrayPanel;
      this.myGUI = myGUI;
      this.dataBuffer = scs.getDataBuffer();
      
      this.dataFileChooser = new JFileChooser();
      this.dataFileChooser.setAcceptAllFileFilterUsed(false);
      this.dataFileChooser.addChoosableFileFilter(dataFileFilter);
      this.accessory = new JPanel();
      this.saveAllGraphs = new JCheckBox("Save graphs in all graph windows");
      this.accessory.setLayout(new BorderLayout());
      this.accessory.add(this.saveAllGraphs, BorderLayout.SOUTH);
      this.dataFileChooser.setAccessory(accessory);
   }

   @Override
   public void constructDialog()
   {
      guiEnablerAndDisabler.disableGUIComponents();

      this.saveAllGraphs.setSelected(false);
      if (dataFileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         chosenFile = dataFileChooser.getSelectedFile();

         String filename = chosenFile.getName();

         if (!filename.endsWith(fileEnding))
         {
            chosenFile = new File(chosenFile.getParent(), filename.concat(fileEnding));
         }
         
         ArrayList<DataEntry> entriesToExport = new ArrayList<>();
         
         if(saveAllGraphs.isSelected())
         {
            for (YoGraph graph : myGUI.getGraphArrayPanel().getGraphsOnThisPanel())
            {
               entriesToExport.addAll(graph.getEntriesOnThisGraph());
            }
            
            for(GraphArrayWindow graphArrayWindow : myGUI.getGraphArrayWindows())
            {
               for (YoGraph graph : graphArrayWindow.getGraphArrayPanel().getGraphsOnThisPanel())
               {
                  entriesToExport.addAll(graph.getEntriesOnThisGraph());
               }   
            }
         }
         else
         {
            for (YoGraph graph : graphArrayPanel.getGraphsOnThisPanel())
            {
               entriesToExport.addAll(graph.getEntriesOnThisGraph());
            }
         }
         
         PrintWriter writer = FileTools.newPrintWriter(chosenFile.toPath());
         
         for (DataEntry dataEntry : entriesToExport)
         {
            writer.print(dataEntry.getVariableName() + ",");
         }
         
         writer.println();
         
         for (int i = dataBuffer.getInPoint(); i < dataBuffer.getOutPoint(); i++)
         {
            for (DataEntry dataEntry : entriesToExport)
            {
               writer.print(dataEntry.getData()[i] + ",");
            }
            
            writer.println();
         }
         
         writer.close();
         
      }
      guiEnablerAndDisabler.enableGUIComponents();
   }

   @Override
   public void closeAndDispose()
   {
      dataFileFilter = null;
      chosenFile = null;
      dataFileChooser = null;
      frame = null;
      myGUI = null;
      
      accessory = null;
      saveAllGraphs = null; 
   }
}
