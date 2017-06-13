package us.ihmc.simulationconstructionset.gui.actions.configActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectGraphGroupAction extends SCSAction
{
   private String name;
   private GraphGroupSelector selector;

   public SelectGraphGroupAction(GraphGroupSelector selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Select: "+name,
              "Select Graph Group: "+name
      );

      this.selector = selector;
      this.name = name;
   }

   @Override
   public void doAction()
   {
      selector.selectGraphGroup(name);
   }
}
