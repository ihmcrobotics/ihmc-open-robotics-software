package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class GotoOutPointAction extends AbstractAction
{
   private static final long serialVersionUID = 4300972053727473361L;

   private GotoOutPointCommandExecutor executor;

   public GotoOutPointAction(GotoOutPointCommandExecutor executor)
   {
      super("Goto Out Point");
      this.executor = executor;

      String iconFilename = "icons/YoGoOutPoint24_2.gif";
      int shortKey = KeyEvent.VK_O;
      String longDescription = "Goto Out Point";
      String shortDescription = "Goto Out Point";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.gotoOutPoint();
   }
}
