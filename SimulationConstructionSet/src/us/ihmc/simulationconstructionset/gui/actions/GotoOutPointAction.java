package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class GotoOutPointAction extends SCSAction
{
   private static final long serialVersionUID = 4300972053727473361L;

   private GotoOutPointCommandExecutor executor;

   public GotoOutPointAction(GotoOutPointCommandExecutor executor)
   {
      super("Goto Out Point",
              "icons/YoGoOutPoint24_2.gif",
              KeyEvent.VK_O,
              "Goto Out Point",
              "Goto Out Point"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.gotoOutPoint();
   }
}
