package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PelvisControlManager implements JumpControlManagerInterface
{
   private final ReferenceFrame controlFrame;
   private final RigidBody pelvis;
   private final ReferenceFrame pelvisFrame;
   private final RigidBody elevator;
   private final ReferenceFrame elevatorFrame;

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand;

   private final FramePoint3D desiredPelvisPosition = new FramePoint3D();
   private final FrameQuaternion desiredPelvisOrientation = new FrameQuaternion();

   private final Twist tempTwist = new Twist();
   private PID3DGainsReadOnly linearGains;
   private PID3DGainsReadOnly angularGains;
   private Vector3DReadOnly linearWeights;
   private Vector3DReadOnly angularWeights;
   private Vector3D zeroWeight = new Vector3D(0.0, 0.0, 0.0);

   public PelvisControlManager(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      controlFrame = ReferenceFrame.getWorldFrame();
      spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      pelvis = fullRobotModel.getPelvis();
      pelvisFrame = pelvis.getBodyFixedFrame();
      elevator = fullRobotModel.getElevator();
      elevatorFrame = elevator.getBodyFixedFrame();

      spatialFeedbackControlCommand.set(elevator, pelvis);
      spatialFeedbackControlCommand.setControlBaseFrame(elevatorFrame);
      spatialFeedbackControlCommand.setSelectionMatrixToIdentity();
   }

   public void setGains(PID3DGainsReadOnly linearGains, PID3DGainsReadOnly angularGains)
   {
      this.linearGains = linearGains;
      this.angularGains = angularGains;
   }

   public void setWeights(Vector3DReadOnly linearWeights, Vector3DReadOnly angularWeights)
   {
      this.linearWeights = linearWeights;
      this.angularWeights = angularWeights;
   }

   public void maintainDesiredOrientationOnly()
   {
      spatialFeedbackControlCommand.set(desiredPelvisPosition);
      spatialFeedbackControlCommand.set(desiredPelvisOrientation);
      spatialFeedbackControlCommand.setPositionGains(linearGains);
      spatialFeedbackControlCommand.setOrientationGains(angularGains);
      spatialFeedbackControlCommand.setWeightsForSolver(zeroWeight, angularWeights);
   }

   public void maintainDesiredPositionAndOrientation()
   {
      spatialFeedbackControlCommand.set(desiredPelvisPosition);
      spatialFeedbackControlCommand.set(desiredPelvisOrientation);
      spatialFeedbackControlCommand.setPositionGains(linearGains);
      spatialFeedbackControlCommand.setOrientationGains(angularGains);
      spatialFeedbackControlCommand.setWeightsForSolver(angularWeights, linearWeights);
   }

   public void getCurrentPelvisAngularVelocity(ReferenceFrame referenceFrame, FrameVector3D angularVelocityToSet)
   {
      tempTwist.set(pelvis.getBodyFixedFrame().getTwistOfFrame());
      tempTwist.changeFrame(referenceFrame);
      tempTwist.getAngularPart(angularVelocityToSet);
   }

   RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void getCurrentPelvisOrientation(ReferenceFrame referenceFrame, FrameQuaternion orientationToSet)
   {
      pelvis.getBodyFixedFrame().getTransformToDesiredFrame(tempTransform, referenceFrame);
      orientationToSet.setIncludingFrame(referenceFrame, tempTransform.getRotationMatrix());
   }

   public void getCurrentPelvisLinearVelocity(ReferenceFrame referenceFrame, FrameVector3D velocityToSet)
   {
      tempTwist.set(pelvis.getBodyFixedFrame().getTwistOfFrame());
      tempTwist.changeFrame(referenceFrame);
      tempTwist.getLinearPart(velocityToSet);
   }

   public void getCurrentPelvisPosition(ReferenceFrame referenceFrame, FramePoint3D positonToSet)
   {
      positonToSet.setToZero(pelvis.getBodyFixedFrame());
      positonToSet.changeFrame(referenceFrame);
   }

   public void setDesiredPelvisPosition(FramePoint3D pelvisPosition)
   {
      this.desiredPelvisPosition.setIncludingFrame(pelvisPosition);
      this.desiredPelvisPosition.changeFrame(controlFrame);
   }

   public void setDesiredPelvisOrientation(FrameQuaternion pelvisOrientation)
   {
      this.desiredPelvisOrientation.setIncludingFrame(pelvisOrientation);
      this.desiredPelvisOrientation.changeFrame(controlFrame);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }
   
   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return spatialFeedbackControlCommand;
   }
}
