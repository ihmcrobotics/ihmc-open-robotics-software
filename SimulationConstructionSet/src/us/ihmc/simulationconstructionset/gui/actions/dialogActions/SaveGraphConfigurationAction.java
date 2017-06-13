package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class SaveGraphConfigurationAction extends SCSAction
{
   private SaveGraphConfigurationDialogConstructor constructor;

   public SaveGraphConfigurationAction(SaveGraphConfigurationDialogConstructor constructor)
   {
      super("Save GraphGroup Configuration",
              "",
              KeyEvent.VK_UNDEFINED,
              "Save GraphGroup Config",
              "Save GraphGroup Configuration"
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
