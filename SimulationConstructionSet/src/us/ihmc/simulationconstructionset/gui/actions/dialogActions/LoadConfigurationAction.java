package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadConfigurationDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class LoadConfigurationAction extends SCSAction
{
   private LoadConfigurationDialogConstructor constructor;
   
   public LoadConfigurationAction(LoadConfigurationDialogConstructor constructor)
   {
      super("Load Configuration...",
              "",
              KeyEvent.VK_UNDEFINED,
              "Load Config",
              "Load Configuration"
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

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }

   public void loadGUIConfigurationFile(File file)
   {
      constructor.loadGUIConfigurationFile(file);
   }
}
