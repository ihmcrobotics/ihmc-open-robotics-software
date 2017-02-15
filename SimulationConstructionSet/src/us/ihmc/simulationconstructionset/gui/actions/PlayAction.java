package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.PlayCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class PlayAction extends AbstractAction
{
   private static final long serialVersionUID = -5829585200441213981L;

   private PlayCommandExecutor executor;

   public PlayAction(PlayCommandExecutor executor)
   {
      super("Play");
      this.executor = executor;

      String iconFilename = "icons/Play24.gif";
      int shortKey = KeyEvent.VK_P;
      String longDescription = "Start playing simulation.";
      String shortDescription = "Play";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.play();
   }
}
