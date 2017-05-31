package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class SelectGraphConfigurationAction extends SCSAction
{
   private static final long serialVersionUID = -8579530834957317679L;
   private String name;
   private SelectGraphConfigurationCommandExecutor executor;

   public SelectGraphConfigurationAction(SelectGraphConfigurationCommandExecutor executor, String name)
   {
      super(name,
              "",
              KeyEvent.VK_UNDEFINED,
              "Select "+name,
              "Select Graph Configuration: "+name
      );

      this.executor = executor;
      this.name = name;
   }

   public void doAction()
   {
      executor.selectGraphConfiguration(name);
   }
}
