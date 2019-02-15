package us.ihmc.humanoidBehaviors.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIMessagerAPI;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
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
   private HumanoidReferenceFrames humanoidReferenceFrames;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabledTopic, placeWaypointA.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabledTopic, placeWaypointB.disableProperty());
   }

   public void setFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);
   }

   @FXML public void homeAll()
   {

   }

   @FXML public void freeze()
   {

   }

   @FXML public void standPrep()
   {

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
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, buildFlatGround());
   }

   @FXML public void placeWaypointA()
   {
      messager.submitMessage(BehaviorUIMessagerAPI.WaypointAPositionEditModeEnabledTopic, true);
      messager.submitMessage(BehaviorUIMessagerAPI.EditModeEnabledTopic, true);
   }

   @FXML public void placeWaypointB()
   {
      messager.submitMessage(BehaviorUIMessagerAPI.WaypointBPositionEditModeEnabledTopic, true);
      messager.submitMessage(BehaviorUIMessagerAPI.EditModeEnabledTopic, true);
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
