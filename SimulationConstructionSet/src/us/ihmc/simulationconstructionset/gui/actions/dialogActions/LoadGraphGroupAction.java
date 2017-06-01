package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadGraphGroupDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class LoadGraphGroupAction extends SCSAction
{
   private LoadGraphGroupDialogConstructor constructor;
   
   public LoadGraphGroupAction(LoadGraphGroupDialogConstructor constructor)
   {
      super("Load Graph Group...",
              "",
              KeyEvent.VK_UNDEFINED,
              "Load Graphs",
              "Load Graph Group"
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

   // TODO: never used?
   public void loadGUIConfigurationFile(File file)
   {
      constructor.loadGraphGroupFile(file);
   }
}
