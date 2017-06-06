package us.ihmc.simulationconstructionset.gui.actions.configActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectVarGroupAction extends SCSAction
{
   private String name;
   private VarGroupSelector varGroupSelector;

   public SelectVarGroupAction(VarGroupSelector selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Select: "+name,
              "Select Var Group: "+name
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
