package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class ZoomInAction extends AbstractAction
{
   private static final long serialVersionUID = -7575893287036365108L;

   private ZoomGraphCommandExecutor executor;

   public ZoomInAction(ZoomGraphCommandExecutor executor)
   {
      super("Zoom In");
      this.executor = executor;

      String iconFilename = "icons/ZoomIn24.gif";
      int shortKey = KeyEvent.VK_I;
      String longDescription = "Zoom In";
      String shortDescription = "Zoom In";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      executor.zoomIn();
   }
   
   public void closeAndDispose()
   {
      executor = null;
   }
}
