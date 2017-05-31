package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SelectGUIConfigFromFileCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class SelectGUIConfigFromFileAction extends SCSAction
{
   private static final long serialVersionUID = -8579530834957317679L;
   private final String fullPath;

   private final SelectGUIConfigFromFileCommandExecutor executor;

   public SelectGUIConfigFromFileAction(String fullPath, String name, SelectGUIConfigFromFileCommandExecutor executor)
   {
      super(name,
              "",
              KeyEvent.VK_UNDEFINED,
              "", // TODO
              "" // TODO
      );

      this.fullPath = fullPath;
      this.executor = executor;
   }

   public void doAction()
   {
      executor.selectGUIConfigFromFile(fullPath);
   }
}
