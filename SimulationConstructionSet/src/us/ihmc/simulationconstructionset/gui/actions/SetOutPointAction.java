package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class SetOutPointAction extends AbstractAction
{
   private static final long serialVersionUID = 3859062359920445441L;

   private SetOutPointCommandExecutor executor;

   public SetOutPointAction(SetOutPointCommandExecutor executor)
   {
      super("Set Out Point");
      this.executor = executor;

      String iconFilename = "icons/YoSetOutPoint24.gif";
      int shortKey = KeyEvent.VK_U;
      String longDescription = "Set Out Point";
      String shortDescription = "Set Out Point";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.setOutPoint();
   }
}
