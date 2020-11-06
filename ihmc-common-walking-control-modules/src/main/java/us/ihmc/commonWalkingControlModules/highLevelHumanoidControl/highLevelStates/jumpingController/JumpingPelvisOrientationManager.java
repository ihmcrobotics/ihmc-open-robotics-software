package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith.PelvisLeapOfFaithModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class JumpingPelvisOrientationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean doPrepareForLocomotion = new YoBoolean("doPreparePelvisForLocomotion", registry);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameQuaternion desiredPelvisOrientation = new FrameQuaternion();
   private final FrameVector3D desiredPelvisAngularVelocity = new FrameVector3D();
   private final FrameVector3D desiredPelvisAngularAcceleration = new FrameVector3D();
   private final FrameQuaternion desiredPelvisOrientationWithOffset = new FrameQuaternion();

   private final FrameQuaternion initialPelvisOrientation = new FrameQuaternion();
   private final FrameQuaternion finalPelvisOrientation = new FrameQuaternion();

   private final SimpleOrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;

   private final YoDouble initialPelvisOrientationTime = new YoDouble("initialPelvisOrientationTime", registry);
   private final YoDouble initialPelvisOrientationOffsetTime = new YoDouble("initialPelvisOrientationOffsetTime", registry);
   private final YoDouble yoTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private Vector3DReadOnly pelvisAngularWeight = null;
   private final Vector3D tempWeight = new Vector3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FrameVector3D tempAngularAcceleration = new FrameVector3D();

   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;
   private final ReferenceFrame midFeetZUpGroundFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final PID3DGainsReadOnly gains;

   private Footstep nextFootstep;
   private final ReferenceFrame nextSoleZUpFrame;
   private final ReferenceFrame nextSoleFrame;

   public JumpingPelvisOrientationManager(PID3DGainsReadOnly gains,
                                          JumpingControllerToolbox controllerToolbox,
                                          YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      midFeetZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();
      pelvisFrame = referenceFrames.getPelvisFrame();
      yoTime = controllerToolbox.getYoTime();

      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);

      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      orientationFeedbackControlCommand.set(elevator, pelvis);
      selectionMatrix.resetSelection();

      desiredPelvisFrame = new ReferenceFrame("desiredPelvisFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            pelvisFrame.getTransformToDesiredFrame(transformToParent, getParent());
            transformToParent.getRotation().set(desiredPelvisOrientation);
         }
      };


      nextSoleFrame = new ReferenceFrame("nextSoleFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            nextFootstep.getSoleReferenceFrame().getTransformToDesiredFrame(transformToParent, getParent());
         }
      };
      nextSoleZUpFrame = new ZUpFrame(worldFrame, nextSoleFrame, "nextAnkleZUp");

      pelvisOrientationOffsetTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvisOffset", false, desiredPelvisFrame, registry);


      parentRegistry.addChild(registry);
   }

   public void setPrepareForLocomotion(boolean value)
   {
      doPrepareForLocomotion.set(value);
   }

   public void compute()
   {
      double deltaTime = yoTime.getDoubleValue() - initialPelvisOrientationTime.getDoubleValue();
      pelvisOrientationTrajectoryGenerator.compute(deltaTime);
      pelvisOrientationTrajectoryGenerator.getAngularData(desiredPelvisOrientation, desiredPelvisAngularVelocity, desiredPelvisAngularAcceleration);

      desiredPelvisOrientation.changeFrame(worldFrame);
      desiredPelvisAngularVelocity.changeFrame(worldFrame);
      desiredPelvisAngularAcceleration.changeFrame(worldFrame);
      desiredPelvisFrame.update();

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      pelvisOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      pelvisOrientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      tempWeight.set(pelvisAngularWeight);


      desiredPelvisOrientationWithOffset.setIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      orientationFeedbackControlCommand.setInverseDynamics(desiredPelvisOrientationWithOffset, desiredPelvisAngularVelocity, desiredPelvisAngularAcceleration);
      orientationFeedbackControlCommand.setWeightsForSolver(tempWeight);
      orientationFeedbackControlCommand.setGains(gains);
      orientationFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
   }

   public void initialize()
   {
      resetOrientationOffset();
      setToZeroInMidFeetZUpFrame();
   }

   public void prepareForLocomotion(double trajectoryTime)
   {
      if (doPrepareForLocomotion.getValue())
      {
         goToHomeFromCurrentDesired(trajectoryTime);
      }
   }

   public void initializeTrajectory()
   {
      updateTrajectoryFromFootstep();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      ret.addCommand(getFeedbackControlCommand());

      return ret;
   }

   public void setWeights(Vector3DReadOnly pelvisAngularWeight)
   {
      this.pelvisAngularWeight = pelvisAngularWeight;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      if (trajectoryTime < 0.0)
      {
         throw new RuntimeException("Negative trajectory time: " + trajectoryTime);
      }
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
   }

   private void initialize(ReferenceFrame desiredTrajectoryFrame)
   {
      initializeTrajectoryFrame(desiredTrajectoryFrame);
      initializeTiming();
   }

   private void initializeTrajectoryFrame(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientation.changeFrame(desiredTrajectoryFrame);
      finalPelvisOrientation.changeFrame(desiredTrajectoryFrame);

      pelvisOrientationTrajectoryGenerator.setReferenceFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setInitialOrientation(initialPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.setFinalOrientation(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.initialize();

      desiredPelvisOrientation.setIncludingFrame(initialPelvisOrientation);
      desiredPelvisOrientation.changeFrame(worldFrame);
      desiredPelvisFrame.update();
   }

   private void initializeTiming()
   {
      initialPelvisOrientationTime.set(yoTime.getDoubleValue());
   }


   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      pelvisOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);

      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void goToHomeFromOffset(FrameQuaternionReadOnly offset, double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setOffset(FrameQuaternionReadOnly offset)
   {
      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setToHoldCurrentInWorldFrame()
   {
      setToHoldCurrent(worldFrame);
   }

   public void setToHoldCurrent(ReferenceFrame trajectoryFrame)
   {
      tempOrientation.setToZero(pelvisFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.setIncludingFrame(tempOrientation);
      finalPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisOrientation.setIncludingFrame(tempOrientation);

      resetOrientationOffset();
      initialize(trajectoryFrame);
   }

   public void centerInMidFeetZUpFrame(double trajectoryTime)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      finalPelvisOrientation.setToZero(midFeetZUpGroundFrame);
      setTrajectoryTime(trajectoryTime);
      initialize(midFeetZUpGroundFrame);
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      setToHoldCurrentDesired(midFeetZUpGroundFrame);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      setToHoldCurrentDesired(soleZUpFrames.get(supportSide));
   }

   public void setToHoldCurrentDesired(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      finalPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      initialize(desiredTrajectoryFrame);
   }

   /**
    * Go instantly to zero, no smooth interpolation.
    */
   public void setToZeroInMidFeetZUpFrame()
   {
      tempOrientation.setToZero(midFeetZUpGroundFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.setIncludingFrame(tempOrientation);
      finalPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisOrientation.setIncludingFrame(tempOrientation);

      initialize(midFeetZUpGroundFrame);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);

      ReferenceFrame otherAnkleZUpFrame = soleZUpFrames.get(supportSide.getOppositeSide());
      ReferenceFrame supportAnkleZUpFrame = soleZUpFrames.get(supportSide);

      tempOrientation.setToZero(otherAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawOtherFoot = tempOrientation.getYaw();

      tempOrientation.setToZero(supportAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawOtherFoot, yawSupportFoot);
      finalPelvisOrientation.setYawPitchRollIncludingFrame(worldFrame, finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(supportAnkleZUpFrame);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      nextFootstep = upcomingFootstep;

      nextSoleFrame.update();
      nextSoleZUpFrame.update();
   }

   public void setTrajectoryFromFootstep()
   {
      updateTrajectoryFromFootstep();

      initializeTiming();
   }

   public void updateTrajectoryFromFootstep()
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);

      RobotSide upcomingFootstepSide = nextFootstep.getRobotSide();
      ReferenceFrame supportSoleFrame = soleZUpFrames.get(upcomingFootstepSide.getOppositeSide());

      nextFootstep.getOrientation(tempOrientation);
      tempOrientation.changeFrame(worldFrame);
      double yawFootstep = tempOrientation.getYaw();

      tempOrientation.setToZero(supportSoleFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawFootstep, yawSupportFoot);
      finalPelvisOrientation.setYawPitchRollIncludingFrame(worldFrame, finalDesiredPelvisYawAngle, 0.0, 0.0);

      initializeTrajectoryFrame(worldFrame);
   }

   public void initializeStanding()
   {
      setToHoldCurrentDesiredInMidFeetZUpFrame();
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}
