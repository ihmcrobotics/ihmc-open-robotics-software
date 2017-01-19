package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;

public class SelectExtraPanelAction extends AbstractAction
{
   private static final long serialVersionUID = -6442315727953394627L;
   private String name;
   private ExtraPanelSelector selectExtraPanelAction;

   public SelectExtraPanelAction(ExtraPanelSelector selector, String name)
   {
      super(name);

      this.selectExtraPanelAction = selector;
      this.name = name;

      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      selectExtraPanelAction.selectPanel(name);
   }
}
