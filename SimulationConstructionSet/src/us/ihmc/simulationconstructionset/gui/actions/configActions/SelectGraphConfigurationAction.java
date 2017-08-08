package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;

public class SelectGraphConfigurationAction extends AbstractAction
{
   private static final long serialVersionUID = -8579530834957317679L;
   private String name;
   private SelectGraphConfigurationCommandExecutor executor;

   public SelectGraphConfigurationAction(SelectGraphConfigurationCommandExecutor executor, String name)
   {
      super(name);

      this.executor = executor;
      this.name = name;

      this.putValue(Action.LONG_DESCRIPTION, "Select Graph Configuration: " + name);
      this.putValue(Action.SHORT_DESCRIPTION, "Select " + name);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.selectGraphConfiguration(name);
   }
}
