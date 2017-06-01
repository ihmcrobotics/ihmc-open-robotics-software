package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveRobotConfigurationDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class SaveRobotConfigurationAction extends SCSAction
{
   private SaveRobotConfigurationDialogConstructor constructor;

   public SaveRobotConfigurationAction(SaveRobotConfigurationDialogConstructor constructor) 
   {
      super("Save Robot Configuration",
              "",
              KeyEvent.VK_UNDEFINED,
              "Save Robot Config",
              "Save Robot Configuration"
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
