package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.ExtraPanelConfiguration;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;

public class SelectExtraPanelAction extends SCSAction
{
   private static final long serialVersionUID = -6442315727953394627L;
   private ExtraPanelConfiguration extraPanelConfiguration;
   private ExtraPanelSelector selectExtraPanelAction;

   public SelectExtraPanelAction(ExtraPanelSelector selector, ExtraPanelConfiguration extraPanelConfiguration)
   {
      super(extraPanelConfiguration.getName(),
              "",
              KeyEvent.VK_E,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.selectExtraPanelAction = selector;
      this.extraPanelConfiguration = extraPanelConfiguration;
   }

   public void doAction()
   {
      selectExtraPanelAction.selectPanel(extraPanelConfiguration.getName());
   }
}
