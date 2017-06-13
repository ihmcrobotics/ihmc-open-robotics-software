package us.ihmc.simulationconstructionset.gui.actions.configActions;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectViewportAction extends SCSAction
{
   private String name;

   private ViewportSelectorCommandExecutor selector;

   public SelectViewportAction(ViewportSelectorCommandExecutor selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Select: "+name,
              "Select Viewport: "+name
      );

      this.selector = selector;
      this.name = name;
   }

   @Override
   public void doAction()
   {
      selector.selectViewport(name);
   }
}
