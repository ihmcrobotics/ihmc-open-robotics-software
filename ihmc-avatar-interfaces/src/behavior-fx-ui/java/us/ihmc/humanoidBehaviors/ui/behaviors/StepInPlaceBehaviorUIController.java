package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import us.ihmc.humanoidBehaviors.StepInPlaceBehavior;
import us.ihmc.messager.Messager;

public class StepInPlaceBehaviorUIController
{
   private Messager behaviorMessager;

   public void init(Messager behaviorMessager)
   {
      this.behaviorMessager = behaviorMessager;
   }

   @FXML public void startStepping()
   {
      behaviorMessager.submitMessage(StepInPlaceBehavior.API.Stepping, true);
   }

   @FXML public void pauseStepping()
   {
      behaviorMessager.submitMessage(StepInPlaceBehavior.API.Stepping, false);
   }
}
