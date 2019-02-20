package us.ihmc.humanoidBehaviors.ui.behaviors;

import controller_msgs.msg.dds.GoHomeMessage;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.paint.Color;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.SimpleMessagerAPIFactory;
import us.ihmc.humanoidBehaviors.ui.graphics.SnappedPositionGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class PatrolBehaviorUIController extends FXUIBehavior
{
   @FXML private Button homeAll;
   @FXML private Button freeze;
   @FXML private Button standPrep;
   @FXML private Button continuePatrol;
   @FXML private Button pausePatrol;
   @FXML private Button destroyPatrol;
   @FXML private Button clearFlat;
   @FXML private Button placeWaypointA;
   @FXML private Button placeWaypointB;

   private JavaFXMessager messager;
   private RobotLowLevelMessenger robotLowLevelMessenger;

   private HumanoidReferenceFrames humanoidReferenceFrames;

   private SnappedPositionGraphic waypointOneGraphic;
   private SnappedPositionGraphic waypointTwoGraphic;

   public PatrolBehaviorUIController()
   {
      // created by JavaFX
   }

   /**
    * Nece
    * @param messager
    */
   public void init(JavaFXMessager messager)
   {
      this.messager = messager;

      waypointOneGraphic = new SnappedPositionGraphic(messager, Color.GREEN);
      waypointTwoGraphic = new SnappedPositionGraphic(messager, Color.YELLOW);

      registerGraphic(waypointOneGraphic);
      registerGraphic(waypointTwoGraphic);
   }

   public void setFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);
   }

   public void setRobotLowLevelMessenger(RobotLowLevelMessenger robotLowLevelMessenger)
   {
      this.robotLowLevelMessenger = robotLowLevelMessenger;
      updateButtons();
   }

   private void updateButtons()
   {
      homeAll.setDisable(messager == null);
      freeze.setDisable(robotLowLevelMessenger == null);
      standPrep.setDisable(robotLowLevelMessenger == null);
   }

   @FXML public void homeAll()
   {
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
//      messager.submitMessage(BehaviorUIMessagerAPI.GoHomeTopic, homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
//      messager.submitMessage(BehaviorUIMessagerAPI.GoHomeTopic, homeRightArm);
   }

   @FXML public void freeze()
   {
      robotLowLevelMessenger.sendFreezeRequest();
   }

   @FXML public void standPrep()
   {
      robotLowLevelMessenger.sendStandRequest();
   }

   @FXML public void continuePatrol()
   {

   }

   @FXML public void pausePatrol()
   {

   }

   @FXML public void destroyPatrol()
   {

   }

   @FXML public void clearFlat()
   {
//      messager.submitMessage(BehaviorUIMessagerAPI.PlanarRegionDataTopic, buildFlatGround());
   }

   @FXML public void placeWaypointA()
   {
      LogTools.debug("placeWaypointA");
      messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypointOneGraphic);
   }

   @FXML public void placeWaypointB()
   {
      LogTools.debug("placeWaypointB");
      messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypointTwoGraphic);
   }

   public static class API
   {
      private static final SimpleMessagerAPIFactory apiFactory = new SimpleMessagerAPIFactory(BehaviorUI.class);


      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
