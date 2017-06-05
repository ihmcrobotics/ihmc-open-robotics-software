package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import javafx.application.Platform;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.dialogs.PlaybackPropertiesDialog;

import javax.swing.*;
import java.awt.*;

public class PlaybackPropertiesDialogGenerator implements PlaybackPropertiesDialogConstructor
{
   private SimulationConstructionSet sim;
   private JFrame frame;
   private Container parentContainer;

   PlaybackPropertiesDialogGenerator(SimulationConstructionSet sim, Container parentContainer, JFrame frame)
   {
      this.sim = sim;
      this.frame = frame;
      this.parentContainer = parentContainer;
   }

   @Override
   public void constructDialog()
   {
      Platform.runLater(() -> new PlaybackPropertiesDialog(parentContainer, frame, sim));
   }

   public void closeAndDispose()
   {
      sim = null;
      frame = null;
      parentContainer = null;
   }
}

