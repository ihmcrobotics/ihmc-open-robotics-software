package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.YoGraphicsPropertiesDialogConstructor;

public class YoGraphicsPropertiesAction extends AbstractAction
{
   private static final long serialVersionUID = -2806456517432086216L;
   private final YoGraphicsPropertiesDialogConstructor constructor;
   
   public YoGraphicsPropertiesAction(YoGraphicsPropertiesDialogConstructor constructor)
   {
      super("YoGraphics Properties...");
      
      this.constructor = constructor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_Y));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      constructor.constructDialog();
   }
}
