package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadRobotConfigurationDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class LoadRobotConfigurationAction extends SCSAction
{
   private LoadRobotConfigurationDialogConstructor constructor;

   public LoadRobotConfigurationAction(LoadRobotConfigurationDialogConstructor constructor)
   {
      super("Load Robot Configuration...",
              "",
              KeyEvent.VK_UNDEFINED,
              "Load Robot Config",
              "Load Robot Configuration"
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
