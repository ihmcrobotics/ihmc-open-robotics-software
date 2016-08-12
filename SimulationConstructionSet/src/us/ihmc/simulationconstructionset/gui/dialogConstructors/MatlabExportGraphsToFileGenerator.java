package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.awt.BorderLayout;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;

import com.jmatio.io.MatFileWriter;
import com.jmatio.types.MLArray;
import com.jmatio.types.MLDouble;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.dataBuffer.DataEntry;
import us.ihmc.simulationconstructionset.gui.GraphArrayPanel;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.MyFileFilter;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoGraph;

public class MatlabExportGraphsToFileGenerator implements ExportGraphsToFileConstructor
{
   private final static String fileEnding = ".mat";

   private javax.swing.filechooser.FileFilter dataFileFilter = new MyFileFilter(new String[] {fileEnding}, "Matlab/octave file (.mat)");
   private File chosenFile;
   private JFileChooser dataFileChooser;
   private JFrame frame;
   private GUIEnablerAndDisabler guiEnablerAndDisabler;
   private GraphArrayPanel graphArrayPanel;
   private StandardSimulationGUI myGUI;

   private JPanel accessory;
   private JCheckBox saveAllGraphs;
   
   private DataBuffer dataBuffer;

   public MatlabExportGraphsToFileGenerator(SimulationConstructionSet scs, JFrame frame, GraphArrayPanel graphArrayPanel, StandardSimulationGUI myGUI)
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
      guiEnablerAndDisabler.enableGUIComponents();
   }

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
