package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import javafx.fxml.FXML;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

public class UIRobotController
{
   private JavaFXMessager messager;

   @FXML
   public void homeAll()
   {
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoHomeTopic, homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoHomeTopic, homeRightArm);
   }

   @FXML
   public void freeze()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelStateMessage.FREEZE_STATE);
      messager.submitMessage(FootstepPlannerMessagerAPI.HighLevelStateTopic, message);
   }

   @FXML
   public void standPrep()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelStateMessage.STAND_PREP_STATE);
      messager.submitMessage(FootstepPlannerMessagerAPI.HighLevelStateTopic, message);
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }
}
