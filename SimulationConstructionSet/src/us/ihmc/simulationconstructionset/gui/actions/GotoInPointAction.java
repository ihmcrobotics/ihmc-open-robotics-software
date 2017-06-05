package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.GotoInPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class GotoInPointAction extends SCSAction
{
   private final GotoInPointCommandExecutor executor;

   public GotoInPointAction(GotoInPointCommandExecutor executor)
   {
      super("Goto In Point",
              "icons/GotoInPoint.png",
              KeyEvent.VK_I,
              "Goto In Point",
              "Goto In Point"
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.gotoInPoint();
   }
}
