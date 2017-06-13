package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SetInPointAction extends SCSAction
{
   private SetInPointCommandExecutor executor;

   public SetInPointAction(SetInPointCommandExecutor executor)
   {
      super("Set In Point",
              "icons/SetInPoint.png",
              KeyEvent.VK_N,
              "Set In Point",
              "Set the current In point in the data buffer."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.setInPoint();
   }
}
