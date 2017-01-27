package us.ihmc.exampleSimulations.beetle.controller;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * Controls the linear Velocity of the body in body z up frame
 * Controls the angular velocity of the body in body frame
 */
public class BodySpatialManager
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final RigidBody[] controlledBodies = new RigidBody[1];
   private final SpatialFeedbackControlCommand spatialFeedbackCommand = new SpatialFeedbackControlCommand();
   private final double controllerDt;
   private final RigidBody body;
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final YoFrameOrientation yoDesiredBodyOrientation;
   private final YoFrameVector yoDesiredBodyLinearVelocity;
   private final YoFrameVector yoDesiredBodyAngularVelocity;
   private final YoFramePoint yoDesiredBodyPosition;
   private final DoubleYoVariable desiredBodyHeight;
   private final AlphaFilteredYoVariable filteredBodyHeight;

   private final Vector3d linearWeight = new Vector3d();
   private final Vector3d angularWeight = new Vector3d();

   public BodySpatialManager(String prefix, FullRobotModel fullRobotModel, HexapodReferenceFrames referenceFrames, double controllerDt,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.controllerDt = controllerDt;
      body = fullRobotModel.getPelvis();
      controlledBodies[0] = body;

      ReferenceFrame bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      ReferenceFrame bodyFixedFrame = body.getBodyFixedFrame();

      yoDesiredBodyPosition = new YoFramePoint(prefix + "desiredPosition", ReferenceFrame.getWorldFrame(), registry);
      yoDesiredBodyLinearVelocity = new YoFrameVector(prefix + "desiredLinearVelocity", bodyZUpFrame, registry);
      yoDesiredBodyOrientation = new YoFrameOrientation(prefix + "desiredOrientation", ReferenceFrame.getWorldFrame(), registry);
      yoDesiredBodyAngularVelocity = new YoFrameVector(prefix + "desiredAngularVelocity", bodyFixedFrame, registry);
      desiredBodyHeight = new DoubleYoVariable(prefix + "desiredBodyHeight", registry);
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, controllerDt);
      filteredBodyHeight = new AlphaFilteredYoVariable("filteredDesiredBodyHeight", registry, alpha, desiredBodyHeight);

      RigidBody elevator = fullRobotModel.getElevator();
      spatialFeedbackCommand.set(elevator, body);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      desiredPosition.setToZero(body.getBodyFixedFrame());
      desiredOrientation.setToZero(body.getBodyFixedFrame());

      spatialFeedbackCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      spatialFeedbackCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);

      yoDesiredBodyOrientation.set(desiredOrientation);
      yoDesiredBodyPosition.set(desiredPosition);
      
      desiredBodyHeight.set(desiredPosition.getZ());
      filteredBodyHeight.set(desiredBodyHeight.getDoubleValue());
   }

   public void doControl(HexapodControllerParameters paramaters)
   {
      spatialFeedbackCommand.setGains(paramaters.getBodySpatialGains());
      paramaters.getBodySpatialLinearQPWeight(linearWeight);
      paramaters.getBodySpatialAngularQPWeight(angularWeight);
      spatialFeedbackCommand.setWeightsForSolver(angularWeight, linearWeight);
      spatialFeedbackCommand.setSelectionMatrix(paramaters.getBodySpatialSelectionMatrix());

      updateDesiredPositionBasedOnDesiredLinearVelocity();
      updateDesiredOrientationBasedOnDesiredAngularVelocity();
      
      filteredBodyHeight.update();
      yoDesiredBodyPosition.setZ(filteredBodyHeight.getDoubleValue());

      yoDesiredBodyPosition.getFrameTupleIncludingFrame(desiredPosition);
      yoDesiredBodyLinearVelocity.getFrameTupleIncludingFrame(desiredLinearVelocity);
      yoDesiredBodyOrientation.getFrameOrientationIncludingFrame(desiredOrientation);
      yoDesiredBodyAngularVelocity.getFrameTupleIncludingFrame(desiredAngularVelocity);

      spatialFeedbackCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      spatialFeedbackCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
   }

 /**
    * we're controlling the yaw rate in world so add our desired rate to the yaw orientation to keep things consistent
    */
   private void updateDesiredOrientationBasedOnDesiredAngularVelocity()
   {
      yoDesiredBodyAngularVelocity.getFrameTupleIncludingFrame(desiredAngularVelocity);
      desiredAngularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      desiredAngularVelocity.scale(controllerDt);
      double deltaYaw = desiredAngularVelocity.getZ();
      yoDesiredBodyOrientation.setYaw(yoDesiredBodyOrientation.getYaw().getDoubleValue() + deltaYaw);
   }

   /**
    * we're controlling the velocity so add our desired velocity to the position to keep things consistent
    */
   private void updateDesiredPositionBasedOnDesiredLinearVelocity()
   {
      yoDesiredBodyLinearVelocity.getFrameTupleIncludingFrame(desiredLinearVelocity);
      desiredLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      desiredLinearVelocity.scale(controllerDt);
      yoDesiredBodyPosition.add(desiredLinearVelocity);
   }

   /**
    * get the command for the controller core
    */
   public SpatialFeedbackControlCommand getSpatialFeedbackControlCommand()
   {
      return spatialFeedbackCommand;
   }

   /**
    * get the command for the controller core
    */
   public SpatialFeedbackControlCommand getFeedbackControlTemplate()
   {
      return spatialFeedbackCommand;
   }

   public RigidBody[] getRigidBodiesToControl()
   {
      return controlledBodies;
   }

   public void getDesiredLinearVelocity(FrameVector desiredLinearVelocityToPack)
   {
      yoDesiredBodyLinearVelocity.getFrameTupleIncludingFrame(desiredLinearVelocityToPack);
   }

   public void getDesiredAngularVelocity(FrameVector desiredAngularVelocityToPack)
   {
      yoDesiredBodyAngularVelocity.getFrameTupleIncludingFrame(desiredAngularVelocityToPack);
      desiredAngularVelocityToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }
}
