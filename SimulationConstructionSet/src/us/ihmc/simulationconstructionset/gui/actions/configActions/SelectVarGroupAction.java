package us.ihmc.simulationconstructionset.gui.actions.configActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;

import java.awt.event.KeyEvent;

public class SelectVarGroupAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      varGroupSelector.selectVarGroup(name);
   }
}
