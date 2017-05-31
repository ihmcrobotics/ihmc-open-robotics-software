package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;


public class SelectViewportAction extends SCSAction
{
   private static final long serialVersionUID = 6565081288826811421L;
   private String name;

   private ViewportSelectorCommandExecutor selector;

   public SelectViewportAction(ViewportSelectorCommandExecutor selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Short Description",
              "Long Description"
      );

      this.selector = selector;
      this.name = name;
   }

   public void doAction()
   {
      selector.selectViewport(name);
   }
}
