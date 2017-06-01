package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

import java.awt.event.KeyEvent;

public class SetOutPointAction extends SCSAction
{
   private SetOutPointCommandExecutor executor;

   public SetOutPointAction(SetOutPointCommandExecutor executor)
   {
      super("Set Out Point",
              "icons/YoSetOutPoint24.gif",
              KeyEvent.VK_U,
              "Set Out Point",
              "Set Out Point"
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.setOutPoint();
   }
}
