package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveConfigurationDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class SaveConfigurationAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

}
