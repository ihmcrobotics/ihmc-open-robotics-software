package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupSelector;


public class SelectEntryBoxGroupAction extends AbstractAction
{
   private static final long serialVersionUID = 2957716760331114426L;
   private String name;
   private EntryBoxGroupSelector selector;

   public SelectEntryBoxGroupAction(EntryBoxGroupSelector selector, String name)
   {
      super(name);

      this.selector = selector;
      this.name = name;

      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      selector.selectEntryBoxGroup(name);
   }
}
