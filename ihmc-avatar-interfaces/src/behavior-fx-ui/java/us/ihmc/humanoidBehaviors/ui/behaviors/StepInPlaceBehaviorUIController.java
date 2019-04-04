package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import us.ihmc.humanoidBehaviors.StepInPlaceBehavior;
import us.ihmc.messager.Messager;

public class StepInPlaceBehaviorUIController
{
   @FXML private Button startStepping;
   @FXML private Button pauseStepping;

   private Messager teleop;

   public void init(Messager behaviorMessager)
   {
      this.teleop = behaviorMessager;
   }

   @FXML public void startStepping()
   {
      teleop.submitMessage(StepInPlaceBehavior.API.Stepping, true);
   }

   @FXML public void pauseStepping()
   {
      teleop.submitMessage(StepInPlaceBehavior.API.Stepping, false);
   }
}
