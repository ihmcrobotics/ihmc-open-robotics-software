package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupSelector;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectEntryBoxGroupAction extends SCSAction
{
   private String name;
   private EntryBoxGroupSelector selector;

   public SelectEntryBoxGroupAction(EntryBoxGroupSelector selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Select: "+name,
              "Select Entry Box Group: "+name
      );

      this.selector = selector;
      this.name = name;
   }

   @Override
   public void doAction()
   {
      selector.selectEntryBoxGroup(name);
   }
}
