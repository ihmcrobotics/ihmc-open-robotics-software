package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.messager.Messager;

public class FancyPosesBehaviorUIController
{
   @FXML
   private CheckBox enableCheckBox;

   @FXML
   private Button singleSupportButton;
   @FXML
   private Button runningManButton;
   @FXML
   private Button karateKid1Button;
   @FXML
   private Button karateKid2Button;
   @FXML
   private Button karateKid3Button;
   @FXML
   private Button shutdownPoseButton;

   private Messager behaviorMessager;

   public void init(Messager behaviorMessager)
   {
      this.behaviorMessager = behaviorMessager;
   }

   @FXML
   public void enable()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Enable, enableCheckBox.isSelected());
   }

   @FXML
   public void requestSingleSupport()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToSingleSupport, true);
   }

   @FXML
   public void requestRunningMan()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToRunningMan, false);
   }

   @FXML
   public void requestKarateKid1()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToKarateKid1, false);
   }

   @FXML
   public void requestKarateKid2()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToKarateKid2, false);
   }

   @FXML
   public void requestKarateKid3()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToKarateKid3, false);
   }

   @FXML
   public void requestPresent()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToPresent, false);
   }

   @FXML
   public void requestShutdownPose()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToShutdownPose, false);
   }
}
