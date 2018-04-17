package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.FileInputStream;
import java.io.IOException;

import javax.swing.JOptionPane;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LoadParametersGenerator implements LoadParametersConstructor
{
   private final SimulationConstructionSet scs;
   private ParameterFileChooser fileChooser;

   
   public LoadParametersGenerator(SimulationConstructionSet scs)
   {
      this.scs = scs;
      this.fileChooser = new ParameterFileChooser();
   }

   @Override
   public void constructDialog()
   {
      
      
      if (fileChooser.showDialog(scs.getJFrame(), scs.getRootRegistry(), scs.getParameterRootPath(), scs.getDefaultParameterFile(), false))
      {
         try
         {
            FileInputStream is = new FileInputStream(fileChooser.getFile());
            XmlParameterReader reader = new XmlParameterReader(is);
            is.close();

            for (YoVariableRegistry child : fileChooser.getRegistries())
            {
               reader.readParametersInRegistry(child);
            }
         }
         catch (IOException e)
         {
            JOptionPane.showMessageDialog(scs.getJFrame(), "Cannot read from " + fileChooser.getFile() + "\n" + e.getMessage(), "Cannot read from file", JOptionPane.ERROR_MESSAGE);
         }
      }
   }

   @Override
   public void closeAndDispose()
   {
      fileChooser.closeAndDispose();
      fileChooser = null;
   }

}
