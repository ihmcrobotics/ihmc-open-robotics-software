package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import us.ihmc.humanoidBehaviors.BehaviorTeleop;
import us.ihmc.humanoidBehaviors.ui.model.FXUIBehavior;

public class StepInPlaceBehaviorUIController extends FXUIBehavior
{
   @FXML private Button startStepping;
   @FXML private Button pauseStepping;

   private BehaviorTeleop teleop;

   public void init(BehaviorTeleop teleop)
   {
      this.teleop = teleop;
   }

   @FXML public void startStepping()
   {
      teleop.setStepping(true);
   }

   @FXML public void pauseStepping()
   {
      teleop.setStepping(false);
   }
}
