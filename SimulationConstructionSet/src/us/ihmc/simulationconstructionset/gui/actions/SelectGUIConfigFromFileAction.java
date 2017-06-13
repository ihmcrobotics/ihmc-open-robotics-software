package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.SelectGUIConfigFromFileCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectGUIConfigFromFileAction extends SCSAction
{
   private final String fullPath;

   private final SelectGUIConfigFromFileCommandExecutor executor;

   public SelectGUIConfigFromFileAction(String fullPath, String name, SelectGUIConfigFromFileCommandExecutor executor)
   {
      super(name,
              "",
              KeyEvent.VK_UNDEFINED,
              "Select: "+fullPath,
              "Select GUI Configuration: "+fullPath
      );

      this.fullPath = fullPath;
      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.selectGUIConfigFromFile(fullPath);
   }
}
