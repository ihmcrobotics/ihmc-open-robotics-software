package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.messager.Messager;

public class FancyPosesBehaviorUIController
{
   @FXML private Button startStepping;
   @FXML private Button pauseStepping;

   private Messager behaviorMessager;

   public void init(Messager behaviorMessager)
   {
      this.behaviorMessager = behaviorMessager;
   }

   @FXML public void startStepping()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, true);
   }

   @FXML public void pauseStepping()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, false);
   }
}
