package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class SetOutPointAction extends SCSAction
{
   private static final long serialVersionUID = 3859062359920445441L;

   private SetOutPointCommandExecutor executor;

   public SetOutPointAction(SetOutPointCommandExecutor executor)
   {
      super("Set Out Point",
              "icons/YoSetOutPoint24.gif",
              KeyEvent.VK_U,
              "Set Out Point",
              "Set Out Point"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.setOutPoint();
   }
}
