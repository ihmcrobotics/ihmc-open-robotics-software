package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogConstructor;

public class SaveGraphConfigurationAction extends SCSAction
{
   private static final long serialVersionUID = 5813345490164040993L;
   private SaveGraphConfigurationDialogConstructor constructor;

   public SaveGraphConfigurationAction(SaveGraphConfigurationDialogConstructor constructor)
   {
      super("Save GraphGroup Configuration",
              "",
              KeyEvent.VK_UNDEFINED,
              "", // TODO
              "" // TODO
      );

      this.constructor = constructor;
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void doAction()
   {
      constructor.constructDialog();
   }
}
