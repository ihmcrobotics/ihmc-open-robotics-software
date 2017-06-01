package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.YoGraphicsPropertiesDialogConstructor;

import java.awt.event.KeyEvent;

public class YoGraphicsPropertiesAction extends SCSAction
{
   private final YoGraphicsPropertiesDialogConstructor constructor;
   
   public YoGraphicsPropertiesAction(YoGraphicsPropertiesDialogConstructor constructor)
   {
      super("YoGraphics Properties...",
              "",
              KeyEvent.VK_Y,
              "Short Description", // TODO
              "Long Description" // TODO
      );
      
      this.constructor = constructor;
   }

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }
}
