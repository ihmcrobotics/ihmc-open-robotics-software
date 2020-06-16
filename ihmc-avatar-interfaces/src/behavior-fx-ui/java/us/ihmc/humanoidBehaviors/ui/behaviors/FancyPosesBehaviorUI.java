package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2NodeInterface;

public class FancyPosesBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(FancyPosesBehavior.DEFINITION, FancyPosesBehaviorUI::new);

   private Messager behaviorMessager;

   @Override
   public void init(SubScene sceneNode, Pane visualizationPane, Ros2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @FXML
   public void requestSingleSupport()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToSingleSupport, true);
   }

   @FXML
   public void requestDoubleSupport()
   {
      behaviorMessager.submitMessage(FancyPosesBehavior.API.GoToDoubleSupport, true);
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
