package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.PlaybackPropertiesDialogConstructor;

public class PlaybackPropertiesAction extends AbstractAction
{
   private static final long serialVersionUID = 4669984581518228636L;

   private final PlaybackPropertiesDialogConstructor constructor;
   
   public PlaybackPropertiesAction(PlaybackPropertiesDialogConstructor constructor)
   {
      super("Playback Properties...");
      
      this.constructor = constructor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_B));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      constructor.constructDialog();
   }
}
