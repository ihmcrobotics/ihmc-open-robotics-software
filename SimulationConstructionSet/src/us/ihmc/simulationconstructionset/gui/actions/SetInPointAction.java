package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

import java.awt.event.KeyEvent;

public class SetInPointAction extends SCSAction
{
   private SetInPointCommandExecutor executor;

   public SetInPointAction(SetInPointCommandExecutor executor)
   {
      super("Set In Point",
              "icons/YoSetInPoint24.gif",
              KeyEvent.VK_N,
              "Set In Point",
              "Set In Point"
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.setInPoint();
   }
}
