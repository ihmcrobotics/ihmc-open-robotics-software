package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupSelector;


public class SelectEntryBoxGroupAction extends SCSAction
{
   private static final long serialVersionUID = 2957716760331114426L;
   private String name;
   private EntryBoxGroupSelector selector;

   public SelectEntryBoxGroupAction(EntryBoxGroupSelector selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.selector = selector;
      this.name = name;
   }

   public void doAction()
   {
      selector.selectEntryBoxGroup(name);
   }
}
