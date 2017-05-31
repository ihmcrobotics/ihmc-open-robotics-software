package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class ZoomInAction extends SCSAction
{
   private static final long serialVersionUID = -7575893287036365108L;

   private ZoomGraphCommandExecutor executor;

   public ZoomInAction(ZoomGraphCommandExecutor executor)
   {
      super("Zoom In",
              "icons/ZoomIn24.gif",
              KeyEvent.VK_I,
              "Zoom In",
              "Zoom In"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.zoomIn();
   }
   
   public void closeAndDispose()
   {
      executor = null;
   }
}
