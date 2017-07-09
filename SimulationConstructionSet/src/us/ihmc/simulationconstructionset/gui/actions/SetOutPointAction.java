package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SetOutPointAction extends SCSAction
{
   private SetOutPointCommandExecutor executor;

   public SetOutPointAction(SetOutPointCommandExecutor executor)
   {
      super("Set Out Point",
              "icons/SetOutPoint.png",
              KeyEvent.VK_U,
              "Set Out Point",
              "Set the current Out point in the data buffer."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.setOutPoint();
   }
}
