package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SelectGUIConfigFromFileCommandExecutor;
public class SelectGUIConfigFromFileAction extends AbstractAction
{
   private static final long serialVersionUID = -8579530834957317679L;
   private final String fullPath;

   private final SelectGUIConfigFromFileCommandExecutor executor;

   public SelectGUIConfigFromFileAction(String fullPath, String name, SelectGUIConfigFromFileCommandExecutor executor)
   {
      super(name);

      this.fullPath = fullPath;
      this.executor = executor;
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.selectGUIConfigFromFile(fullPath);
   }
}
