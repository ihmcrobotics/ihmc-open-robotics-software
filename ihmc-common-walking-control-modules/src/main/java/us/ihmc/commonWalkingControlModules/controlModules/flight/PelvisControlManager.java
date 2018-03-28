package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PelvisControlManager implements JumpControlManagerInterface
{
   private final ReferenceFrame controlFrame;
   private final RigidBody pelvis;
   private final ReferenceFrame pelvisFrame;
   private final RigidBody elevator;
   private final ReferenceFrame elevatorFrame;

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand;

   //private final FrameVector3D pelvisFeedbackLinearAcceleration = new FrameVector3D();
   //private final FrameVector3D pelvisFeedbackAngularAcceleration = new FrameVector3D();

   private final FramePoint3D desiredPelvisPosition = new FramePoint3D();
   //private final FramePoint3D currentPelvisPosition = new FramePoint3D();
   //private final FrameVector3D currentPelvisVelocity = new FrameVector3D();

   private final FrameQuaternion desiredPelvisOrientation = new FrameQuaternion();
   //private final FrameQuaternion currentPelvisOrientation = new FrameQuaternion();
   //private final FrameVector3D currentPelvisAngularVelocity = new FrameVector3D();

   //private final FramePoint3D pelvisPositionError = new FramePoint3D();
   //private final FrameVector3D pelvisOrientationError = new FrameVector3D();

   private final Twist tempTwist = new Twist();
   private EnumProvider<JumpStateEnum> currentState;
   private PID3DGainsReadOnly linearGains;
   private PID3DGainsReadOnly angularGains;
   private Vector3DReadOnly linearWeights;
   private Vector3DReadOnly angularWeights;

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

   public void compute()
   {
      switch (currentState.getValue())
      {
      case STANDING:
         spatialFeedbackControlCommand.set(desiredPelvisPosition);
         spatialFeedbackControlCommand.set(desiredPelvisOrientation);
         spatialFeedbackControlCommand.setPositionGains(linearGains);
         spatialFeedbackControlCommand.setOrientationGains(angularGains);
         spatialFeedbackControlCommand.setWeightsForSolver(angularWeights, linearWeights);
         break;
      case TAKE_OFF:
      case FLIGHT:
      case LANDING:
         throw new RuntimeException("Unimplemented case");
      default:
         throw new RuntimeException("Unknown jump control state");

      }

   }

   //   private void computeMomentumFeedbackForPelvisHeight(FrameVector3D linearFeedbackToSet, FrameVector3D angularFeedbackToSet)
   //   {
   //      getCurrentPelvisPosition(controlFrame, currentPelvisPosition);
   //      getCurrentPelvisLinearVelocity(controlFrame, currentPelvisVelocity);
   //      pelvisPositionError.sub(desiredPelvisPosition, currentPelvisPosition);
   //      pelvisPositionError.scale(0.0, 0.0, 200.0);
   //      currentPelvisVelocity.scale(-28.28);
   //      linearFeedbackToSet.setIncludingFrame(pelvisPositionError);
   //      linearFeedbackToSet.add(currentPelvisVelocity);
   //
   //      getCurrentPelvisOrientation(controlFrame, currentPelvisOrientation);
   //      getCurrentPelvisAngularVelocity(controlFrame, currentPelvisAngularVelocity);
   //      subtractQuaternionsToGetRotationalAxis(desiredPelvisOrientation, currentPelvisOrientation, pelvisOrientationError);
   //      pelvisOrientationError.scale(1.0);
   //      currentPelvisAngularVelocity.scale(-2.0);
   //      angularFeedbackToSet.setIncludingFrame(controlFrame, pelvisOrientationError);
   //      angularFeedbackToSet.add(currentPelvisAngularVelocity);
   //   }

   private void subtractQuaternionsToGetRotationalAxis(FrameQuaternion orientation1, FrameQuaternion orientation2, FrameVector3D rotationalAxisToSet)
   {
      orientation1.checkReferenceFrameMatch(orientation2);
      double ds = orientation1.getS() - orientation2.getS();
      double dx = orientation1.getX() - orientation2.getX();
      double dy = orientation1.getY() - orientation2.getY();
      double dz = orientation1.getZ() - orientation2.getZ();
      double theta = Math.acos(ds);
      double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (length > 1e-10)
         rotationalAxisToSet.setIncludingFrame(orientation1.getReferenceFrame(), theta * dx / length, theta * dy / length, theta * dz / length);
      else
         rotationalAxisToSet.setToZero(orientation1.getReferenceFrame());
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
   public void setStateEnumProvider(EnumProvider<JumpStateEnum> stateEnumProvider)
   {
      this.currentState = stateEnumProvider;
   }
}
