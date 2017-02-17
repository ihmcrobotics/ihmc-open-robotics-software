package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.BorderLayout;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;

import com.jmatio.io.MatFileWriter;
import com.jmatio.types.MLArray;
import com.jmatio.types.MLDouble;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;
import us.ihmc.tools.gui.MyFileFilter;
import us.ihmc.tools.io.files.FileTools;

public class ExportGraphsToFileGenerator implements ExportGraphsToFileConstructor
{
   private final static String matEnding = ".mat";
   private final static String csvEnding = ".csv";

   private javax.swing.filechooser.FileFilter matFileFilter = new MyFileFilter(new String[] {matEnding}, "Matlab/octave file (.mat)");
   private javax.swing.filechooser.FileFilter csvFileFilter = new MyFileFilter(new String[] {csvEnding}, "CSV file (.csv)");
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   private GraphArrayPanel graphArrayPanel;
   private StandardSimulationGUI myGUI;

   private JPanel accessory;
   private JCheckBox saveAllGraphs;
   
   private DataBuffer dataBuffer;

   public ExportGraphsToFileGenerator(SimulationConstructionSet scs, JFrame frame, GraphArrayPanel graphArrayPanel, StandardSimulationGUI myGUI)
   {
      this.frame = frame;
      this.guiEnablerAndDisabler = scs;
      this.graphArrayPanel = graphArrayPanel;
      this.myGUI = myGUI;
      this.dataBuffer = scs.getDataBuffer();
      
      
      
      this.dataFileChooser = new JFileChooser();
      this.dataFileChooser.setAcceptAllFileFilterUsed(false);
      this.dataFileChooser.addChoosableFileFilter(matFileFilter);
      this.dataFileChooser.addChoosableFileFilter(csvFileFilter);
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

         
         
         if(dataFileChooser.getFileFilter() == matFileFilter)
         {
            exportToMAT(dataFileChooser.getSelectedFile(), entriesToExport);
         }
         else if (dataFileChooser.getFileFilter() == csvFileFilter)
         {
            exportToCSV(dataFileChooser.getSelectedFile(), entriesToExport);
         }
         else
         {
            throw new RuntimeException("Unknown data type selected " + dataFileChooser.getFileFilter());
         }
         
      }
      guiEnablerAndDisabler.enableGUIComponents();
   }

   private void exportToCSV(File chosenFile, ArrayList<DataEntry> entriesToExport)
   {
      String filename = chosenFile.getName();

      if (!filename.endsWith(csvEnding))
      {
         chosenFile = new File(chosenFile.getParent(), filename.concat(csvEnding));
      }
      
      
      PrintWriter writer = FileTools.newPrintWriter(chosenFile.toPath(), WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_STACKTRACE);
      
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
   
   private void exportToMAT(File chosenFile, ArrayList<DataEntry> entriesToExport)
   {
      String filename = chosenFile.getName();

      if (!filename.endsWith(matEnding))
      {
         chosenFile = new File(chosenFile.getParent(), filename.concat(matEnding));
      }
      
      
      ArrayList<MLArray>  matlabData = new ArrayList<>();

      for(DataEntry entry : entriesToExport)
      {
         matlabData.add(convertToMatlabArray(entry, dataBuffer.getInPoint(), dataBuffer.getOutPoint()));
      }
      
      try
      {
         new MatFileWriter(chosenFile, matlabData);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public void closeAndDispose()
   {
      matFileFilter = null;
      csvFileFilter = null;
      dataFileChooser = null;
      frame = null;
      myGUI = null;
      
      accessory = null;
      saveAllGraphs = null; 
   }

   public static MLDouble convertToMatlabArray(DataEntry entry, int inPoint, int outPoint)
   {
      YoVariable<?> variable = entry.getVariable();
      
      double[] inData = entry.getData();
      double[] data;
      
      if(inPoint == outPoint)
      {
         // Edge case, export one element
         data = new double[1];
         data[0] = inData[inPoint];
      }
      else if(inPoint < outPoint)
      {
         // Data is not wrapped
         data = new double[outPoint + 1 - inPoint];
         System.arraycopy(inData, inPoint, data, 0, data.length);
      }
      else
      {
         // Data is wrapped
         int length = inData.length - inPoint + outPoint  + 1;
         data = new double[length];
         System.arraycopy(inData, inPoint, data, 0, inData.length - inPoint);
         System.arraycopy(inData, 0, data, inData.length - inPoint, outPoint + 1);
      }
      

      return new MLDouble(variable.getName(), data, 1);
      
   }


}
