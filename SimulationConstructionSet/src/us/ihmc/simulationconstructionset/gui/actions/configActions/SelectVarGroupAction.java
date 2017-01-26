package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;


public class SelectVarGroupAction extends AbstractAction
{
   private static final long serialVersionUID = 7027661415555951268L;
   private String name;
   private VarGroupSelector varGroupSelector;

   public SelectVarGroupAction(VarGroupSelector selector, String name)
   {
      super(name);

      this.varGroupSelector = selector;
      this.name = name;

      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      varGroupSelector.selectVarGroup(name);
   }
}
