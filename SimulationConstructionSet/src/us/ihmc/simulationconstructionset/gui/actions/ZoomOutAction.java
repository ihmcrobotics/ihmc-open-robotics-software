package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class ZoomOutAction extends SCSAction
{
   private ZoomGraphCommandExecutor executor;

   public ZoomOutAction(ZoomGraphCommandExecutor executor)
   {
      super("Zoom Out",
              "icons/ZoomOut24.gif",
              KeyEvent.VK_O,
              "Zoom Out",
              "Zoom Out"
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
