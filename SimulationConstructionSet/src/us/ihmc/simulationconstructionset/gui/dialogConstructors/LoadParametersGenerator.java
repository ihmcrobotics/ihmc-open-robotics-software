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

   public LoadParametersGenerator(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }

   @Override
   public void constructDialog()
   {
      ParameterFileChooser fileChooser = new ParameterFileChooser(scs.getParameterRootPath());

      if (fileChooser.showDialog(scs.getJFrame(), scs.getRootRegistry(), false))
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

      fileChooser.closeAndDispose();
   }

   @Override
   public void closeAndDispose()
   {
      // TODO Auto-generated method stub

   }

}
