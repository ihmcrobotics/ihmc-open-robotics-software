package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.PlayCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class PlayAction extends SCSAction
{
   private static final long serialVersionUID = -5829585200441213981L;

   private PlayCommandExecutor executor;

   public PlayAction(PlayCommandExecutor executor)
   {
      super("Play",
              "icons/Play24.gif",
              KeyEvent.VK_P,
              "Play",
              "Start playing simulation."
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.play();
   }
}
