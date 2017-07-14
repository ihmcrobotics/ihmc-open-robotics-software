package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class ZoomOutAction extends SCSAction
{
   private ZoomGraphCommandExecutor executor;

   public ZoomOutAction(ZoomGraphCommandExecutor executor)
   {
      super("Zoom Out",
              "icons/ZoomOut.png",
              KeyEvent.VK_O,
              "Zoom Out Data Buffer",
              "Shorten graphs to get rougher detail of data buffer."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.zoomOut();
   }
   
   public void closeAndDispose()
   {
      executor = null;
   }
}
