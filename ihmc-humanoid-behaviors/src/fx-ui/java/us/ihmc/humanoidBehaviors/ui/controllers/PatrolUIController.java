package us.ihmc.humanoidBehaviors.ui.controllers;

import controller_msgs.msg.dds.GoHomeMessage;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI.API;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PatrolUIController
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

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

//      messager.bindPropertyToTopic(API.ActiveEditor, placeWaypointA.disableProperty());
//      messager.bindPropertyToTopic(BehaviorUIMessagerAPI.EditModeEnabledTopic, placeWaypointB.disableProperty());
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
//      messager.submitMessage(BehaviorUIMessagerAPI.WaypointAPositionEditModeEnabledTopic, true);
//      messager.submitMessage(BehaviorUIMessagerAPI.EditModeEnabledTopic, true);
      LogTools.debug("placeWaypointA");
      messager.submitMessage(API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      messager.submitMessage(API.SelectedGraphic, BehaviorUI.waypointOneGraphic);
   }

   @FXML public void placeWaypointB()
   {
//      messager.submitMessage(BehaviorUIMessagerAPI.WaypointBPositionEditModeEnabledTopic, true);
//      messager.submitMessage(BehaviorUIMessagerAPI.EditModeEnabledTopic, true);
      messager.submitMessage(API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      messager.submitMessage(API.SelectedGraphic, BehaviorUI.waypointTwoGraphic);
   }

   private PlanarRegionsList buildFlatGround()
   {
      humanoidReferenceFrames.updateFrames();
      RigidBodyTransform transformToWorld = humanoidReferenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame();
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(10.0, 10.0);
      convexPolygon.addVertex(-10.0, 10.0);
      convexPolygon.addVertex(-10.0, -10.0);
      convexPolygon.addVertex(10.0, -10.0);
      convexPolygon.update();
      PlanarRegion groundPlane = new PlanarRegion(transformToWorld, convexPolygon);
      return new PlanarRegionsList(groundPlane);
   }
}
