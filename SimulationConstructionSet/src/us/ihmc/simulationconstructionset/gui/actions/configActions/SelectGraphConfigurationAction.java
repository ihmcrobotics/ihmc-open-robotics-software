package us.ihmc.simulationconstructionset.gui.actions.configActions;

import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectGraphConfigurationAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      executor.selectGraphConfiguration(name);
   }
}
