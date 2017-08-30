package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.CreateNewParameterSliderWindowCommandExecutor;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

/**
 * Created by Peter on 8/29/2017.
 */

public class CreateParameterSliderWindowAction extends AbstractAction
{
   private static final long serialVersionUID = 5063019587274426802L;
   private CreateNewParameterSliderWindowCommandExecutor executor;

   public CreateParameterSliderWindowAction(CreateNewParameterSliderWindowCommandExecutor executor)
   {
      super("New Slider Window");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_S));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.createNewParameterSliderWindow();
   }
}

