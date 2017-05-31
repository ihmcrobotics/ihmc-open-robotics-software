package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.YoGraphicsPropertiesDialogConstructor;

public class YoGraphicsPropertiesAction extends SCSAction
{
   private static final long serialVersionUID = -2806456517432086216L;
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

   public void doAction()
   {
      constructor.constructDialog();
   }
}
