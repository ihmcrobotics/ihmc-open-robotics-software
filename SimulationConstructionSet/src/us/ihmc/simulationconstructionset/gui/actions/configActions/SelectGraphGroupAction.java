package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;


public class SelectGraphGroupAction extends SCSAction
{
   private static final long serialVersionUID = -5534400254284863190L;
   private String name;
   private GraphGroupSelector selector;

   public SelectGraphGroupAction(GraphGroupSelector selector, String name)
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
      selector.selectGraphGroup(name);
   }
}
