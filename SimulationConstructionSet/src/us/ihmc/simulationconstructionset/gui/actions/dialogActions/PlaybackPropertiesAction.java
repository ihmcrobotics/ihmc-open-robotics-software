package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PlaybackPropertiesDialogConstructor;

public class PlaybackPropertiesAction extends SCSAction
{
   private static final long serialVersionUID = 4669984581518228636L;

   private final PlaybackPropertiesDialogConstructor constructor;
   
   public PlaybackPropertiesAction(PlaybackPropertiesDialogConstructor constructor)
   {
      super("Playback Properties...",
              "",
              KeyEvent.VK_B,
              "Short Description", // TODO
              "Long Description" // TODO
      );
      
      this.constructor = constructor;
   }

   public void doAction()
   {
      constructor.constructDialog();
   }
}
