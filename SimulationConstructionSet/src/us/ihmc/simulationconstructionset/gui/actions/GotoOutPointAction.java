package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class GotoOutPointAction extends SCSAction
{
   private GotoOutPointCommandExecutor executor;

   public GotoOutPointAction(GotoOutPointCommandExecutor executor)
   {
      super("Goto Out Point",
              "icons/GotoOutPoint.png",
              KeyEvent.VK_O,
              "Goto Out Point",
              "Goto Out Point"
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.gotoOutPoint();
   }
}
