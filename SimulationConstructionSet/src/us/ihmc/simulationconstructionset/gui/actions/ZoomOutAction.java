package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.ZoomGraphCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class ZoomOutAction extends AbstractAction
{
   private static final long serialVersionUID = 5487975503056238066L;

   private ZoomGraphCommandExecutor executor;

   public ZoomOutAction(ZoomGraphCommandExecutor executor)
   {
      super("Zoom Out");
      this.executor = executor;

      String iconFilename = "icons/ZoomOut24.gif";
      int shortKey = KeyEvent.VK_O;
      String longDescription = "Zoom Out";
      String shortDescription = "Zoom Out";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.zoomOut();
   }
   
   public void closeAndDispose()
   {
      executor = null;
   }
}
