package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogConstructor;

public class SaveGraphConfigurationAction extends AbstractAction
{
   private static final long serialVersionUID = 5813345490164040993L;
   private SaveGraphConfigurationDialogConstructor constructor;

   public SaveGraphConfigurationAction(SaveGraphConfigurationDialogConstructor constructor)
   {
      super("Save GraphGroup Configuration");
      this.constructor= constructor;
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      constructor.constructDialog();
   }
}
