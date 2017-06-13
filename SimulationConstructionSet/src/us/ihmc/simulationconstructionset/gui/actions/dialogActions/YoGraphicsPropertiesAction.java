package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.YoGraphicsPropertiesDialogConstructor;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class YoGraphicsPropertiesAction extends SCSAction
{
   private final YoGraphicsPropertiesDialogConstructor constructor;
   
   public YoGraphicsPropertiesAction(YoGraphicsPropertiesDialogConstructor constructor)
   {
      super("YoGraphics Properties...",
              "",
              KeyEvent.VK_Y,
              "Open YoGraphics Properties",
              "Open YoGraphics Properties for in-depth settings adjustment."
      );
      
      this.constructor = constructor;
   }

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }
}
