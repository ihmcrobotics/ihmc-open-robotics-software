package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PlaybackPropertiesDialogConstructor;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class PlaybackPropertiesAction extends SCSAction
{
   private final PlaybackPropertiesDialogConstructor constructor;
   
   public PlaybackPropertiesAction(PlaybackPropertiesDialogConstructor constructor)
   {
      super("Playback Properties...",
              "",
              KeyEvent.VK_B,
              "Open Playback Properties",
              "Open Playback Properties for in-depth settings adjustment."
      );
      
      this.constructor = constructor;
   }

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }
}
