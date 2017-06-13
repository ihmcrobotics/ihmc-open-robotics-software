package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.ExtraPanelConfiguration;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class SelectExtraPanelAction extends SCSAction
{
   private ExtraPanelConfiguration extraPanelConfiguration;
   private ExtraPanelSelector selectExtraPanelAction;

   public SelectExtraPanelAction(ExtraPanelSelector selector, ExtraPanelConfiguration extraPanelConfiguration)
   {
      super(extraPanelConfiguration.getName(),
              "",
              KeyEvent.VK_E,
              "Select: "+extraPanelConfiguration.getName(),
              "Select Extra Panel: "+extraPanelConfiguration.getName()
      );

      this.selectExtraPanelAction = selector;
      this.extraPanelConfiguration = extraPanelConfiguration;
   }

   @Override
   public void doAction()
   {
      selectExtraPanelAction.selectPanel(extraPanelConfiguration.getName());
   }
}
