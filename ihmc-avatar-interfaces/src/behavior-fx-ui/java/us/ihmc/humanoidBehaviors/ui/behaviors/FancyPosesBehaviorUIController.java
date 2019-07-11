package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.messager.Messager;

public class FancyPosesBehaviorUIController
{
   @FXML private Button pose0Button;
   @FXML private Button pose1Button;
   @FXML private Button pose2Button;
   @FXML private Button pose3Button;
   @FXML private Button pose4Button;

   private Messager behaviorMessager;

   public void init(Messager behaviorMessager)
   {
      this.behaviorMessager = behaviorMessager;
   }

   @FXML public void requestPose0()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, true);
   }

   @FXML public void requestPose1()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, false);
   }
   
   @FXML public void requestPose2()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, false);
   }
   
   @FXML public void requestPose3()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, false);
   }
   
   @FXML public void requestPose4()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.Stepping, false);
   }
}
