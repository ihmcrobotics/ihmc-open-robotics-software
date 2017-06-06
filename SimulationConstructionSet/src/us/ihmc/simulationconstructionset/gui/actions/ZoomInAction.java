package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class ZoomInAction extends SCSAction
{
   private ZoomGraphCommandExecutor executor;

   public ZoomInAction(ZoomGraphCommandExecutor executor)
   {
      super("Zoom In",
              "icons/ZoomIn.png",
              KeyEvent.VK_I,
              "Zoom In Data Buffer",
              "Expand graphs to get finer detail of data buffer."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.zoomIn();
   }
   
   public void closeAndDispose()
   {
      executor = null;
   }
}
