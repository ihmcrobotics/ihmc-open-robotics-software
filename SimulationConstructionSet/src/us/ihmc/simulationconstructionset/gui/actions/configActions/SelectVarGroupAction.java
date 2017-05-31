package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;


public class SelectVarGroupAction extends SCSAction
{
   private static final long serialVersionUID = 7027661415555951268L;
   private String name;
   private VarGroupSelector varGroupSelector;

   public SelectVarGroupAction(VarGroupSelector selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.varGroupSelector = selector;
      this.name = name;
   }

   public void doAction()
   {
      varGroupSelector.selectVarGroup(name);
   }
}
