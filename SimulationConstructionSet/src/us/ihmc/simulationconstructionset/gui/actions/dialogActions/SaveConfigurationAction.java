package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveConfigurationDialogConstructor;

public class SaveConfigurationAction extends SCSAction
{
   private static final long serialVersionUID = 5813345490164040993L;
   private SaveConfigurationDialogConstructor constructor;

   public SaveConfigurationAction(SaveConfigurationDialogConstructor constructor)
   {
      super("Save Configuration",
              "",
              KeyEvent.VK_UNDEFINED,
              "Save Config",
              "Save Configuration"
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
