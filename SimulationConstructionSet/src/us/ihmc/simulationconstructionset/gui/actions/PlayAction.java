package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.PlayCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class PlayAction extends SCSAction
{
   private PlayCommandExecutor executor;

   public PlayAction(PlayCommandExecutor executor)
   {
      super("Play",
              "icons/Play.png",
              KeyEvent.VK_P,
              "Play",
              "Start playing simulation."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.play();
   }
}
