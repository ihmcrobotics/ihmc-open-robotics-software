package us.ihmc.commonWalkingControlModules.controlModules;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class PelvisOrientationManager
{
   private static final double defaultTrajectoryTime = 1.0;

   private static final double pelvisYawAngleDegrees = 40.0; // degrees
   private static final double pelvisPitchRatio = 0.2;
   private static final boolean PELVIS_YAW_FROM_FEET_POSITIONS = true;
   private static final boolean addPelvisOffsetsWhileStepping = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameQuaternion desiredPelvisQuaternion = new YoFrameQuaternion("desiredPelvis", worldFrame, registry);
   private final YoFrameOrientation desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularVelocity = new YoFrameVector("desiredPelvisAngularVelocity", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", worldFrame, registry);

   private final DoubleYoVariable swingPelvisYaw = new DoubleYoVariable("swingPelvisYaw", registry);
   private final DoubleYoVariable swingPelvisYawScale = new DoubleYoVariable("swingPelvisYawScale", registry);

   private final DoubleYoVariable initialPelvisOrientationTime = new DoubleYoVariable("initialPelvisOrientationTime", registry);
   private final YoFrameQuaternion initialPelvisOrientation = new YoFrameQuaternion("initialPelvis", worldFrame, registry);
   private final YoFrameQuaternion finalPelvisOrientation = new YoFrameQuaternion("finalPelvis", worldFrame, registry);
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final DoubleYoVariable initialPelvisOrientationOffsetTime = new DoubleYoVariable("initialPelvisOrientationOffsetTime", registry);

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationOffsetTrajectoryGenerator;

   private final YoFrameOrientation desiredPelvisOffsetWhileWalking = new YoFrameOrientation("desiredPelvisOffsetWhileWalking", worldFrame, registry);
   private final YoFrameQuaternion desiredPelvisQuaternionWithOffset = new YoFrameQuaternion("desiredPelvisOrientationWithOffset", worldFrame, registry);
   private final YoFrameOrientation desiredPelvisOrientationWithOffset = new YoFrameOrientation("desiredPelvisOrientationWithOffset", worldFrame, registry);

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable timeInState = new DoubleYoVariable("pelvisOrientationTimeInState", registry);
   private double initialTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final YoFrameVector yoPelvisAngularWeight = new YoFrameVector("pelvisWeight", null, registry);
   private final Vector3D pelvisAngularWeight = new Vector3D();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isPelvisOrientationOffsetTrajectoryStopped", registry);

   private final YoOrientationPIDGainsInterface gains;

   private final LongYoVariable lastCommandId;

   private final BooleanYoVariable isReadyToHandleQueuedCommands;
   private final LongYoVariable numberOfQueuedCommands;
   private final RecyclingArrayDeque<PelvisOrientationTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisOrientationTrajectoryCommand.class);

   private final BooleanYoVariable isStanding = new BooleanYoVariable("pelvisIsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable("pelvisInInTransfer", registry);

   private final DoubleYoVariable transferDuration = new DoubleYoVariable("pelvisTransferDuration", registry);
   private final DoubleYoVariable swingDuration = new DoubleYoVariable("pelvisSwingDuration", registry);

   private final BooleanYoVariable followPelvisYawSineWave = new BooleanYoVariable("followPelvisYawSineWave", registry);
   private final DoubleYoVariable pelvisYawSineFrequency = new DoubleYoVariable("pelvisYawSineFrequency", registry);
   private final DoubleYoVariable pelvisYawSineMagnitude = new DoubleYoVariable("pelvisYawSineMagnitude", registry);
   private final DoubleYoVariable pelvisYawRatio = new DoubleYoVariable("pelvisYawRatio", registry);
   private final DoubleYoVariable pelvisYawAngle = new DoubleYoVariable("pelvisYawAngle", registry);

   private final BooleanYoVariable addPelvisOffsetsBasedOnStep = new BooleanYoVariable("addPelvisOffsetsBasedOnStep", registry);
   private final DoubleYoVariable pelvisPitchAngleRatio = new DoubleYoVariable("pelvisPitchAngleRatio", registry);

   private RobotSide supportSide;

   private final SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();

   public PelvisOrientationManager(WalkingControllerParameters walkingControllerParameters, HighLevelHumanoidControllerToolbox controllerToolbox,
         YoVariableRegistry parentRegistry)
   {
      yoTime = controllerToolbox.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      pelvisFrame = referenceFrames.getPelvisFrame();

      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);
      double defaultStepTime = walkingControllerParameters.getDefaultSwingTime();
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(defaultStepTime);

      pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         pelvisOrientationTrajectoryGenerator.registerNewTrajectoryFrame(ankleZUpFrames.get(robotSide));

      gains = walkingControllerParameters.createPelvisOrientationControlGains(registry);
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody pelvis = fullRobotModel.getPelvis();
      yoPelvisAngularWeight.set(SolverWeightLevels.PELVIS_WEIGHT, SolverWeightLevels.PELVIS_WEIGHT, SolverWeightLevels.PELVIS_WEIGHT);
      yoPelvisAngularWeight.get(pelvisAngularWeight);
      orientationFeedbackControlCommand.set(elevator, pelvis);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);

      desiredPelvisFrame = new ReferenceFrame("desiredPelvisFrame", worldFrame)
      {
         private static final long serialVersionUID = -1472151257649344278L;

         private final Quaternion rotationToParent = new Quaternion();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            pelvisFrame.getTransformToDesiredFrame(transformToParent, parentFrame);
            desiredPelvisQuaternion.get(rotationToParent);
            transformToParent.setRotation(rotationToParent);
         }
      };

      boolean allowMultipleFrames = true;
      orientationOffsetTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("pelvisOffset", allowMultipleFrames,
            desiredPelvisFrame, registry);
      orientationOffsetTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

      String namePrefix = "pelvisOrientation";
      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new BooleanYoVariable(namePrefix + "IsReadyToHandleQueuedPelvisOrientationTrajectoryCommands", registry);
      numberOfQueuedCommands = new LongYoVariable(namePrefix + "NumberOfQueuedCommands", registry);

      addPelvisOffsetsBasedOnStep.set(addPelvisOffsetsWhileStepping);
      pelvisPitchAngleRatio.set(pelvisPitchRatio);
      pelvisYawAngle.set(pelvisYawAngleDegrees);
      pelvisYawRatio.set(0.2);

      parentRegistry.addChild(registry);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         transformsFromAnkleToSole.put(robotSide, ankleToSole);
      }
   }

   public void setWeight(double weight)
   {
      yoPelvisAngularWeight.set(weight, weight, weight);
   }

   public void setWeights(Vector3D weight)
   {
      yoPelvisAngularWeight.set(weight);
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
   }

   private void initialize(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientationTime.set(yoTime.getDoubleValue());

      pelvisOrientationTrajectoryGenerator.switchTrajectoryFrame(desiredTrajectoryFrame);

      initialPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setInitialOrientation(tempOrientation);

      finalPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setFinalOrientation(tempOrientation);

      pelvisOrientationTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisQuaternion.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
   }

   public void compute()
   {
      double deltaTime = yoTime.getDoubleValue() - initialPelvisOrientationTime.getDoubleValue();

      pelvisOrientationTrajectoryGenerator.compute(deltaTime);
      pelvisOrientationTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisQuaternion.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
      desiredPelvisFrame.update();

      if (followPelvisYawSineWave.getBooleanValue())
      {
         double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(yoTime.getDoubleValue() * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
         tempOrientation.setIncludingFrame(midFeetZUpFrame, yaw, 0.0, 0.0);

         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);
         tempAngularAcceleration.setToZero(worldFrame);

         desiredPelvisQuaternion.set(tempOrientation);
         desiredPelvisOrientation.set(tempOrientation);
         desiredPelvisAngularVelocity.set(tempAngularVelocity);
         desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
         desiredPelvisFrame.update();
      }
      else if (isTrajectoryStopped.getBooleanValue())
      {
         orientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
         tempAngularVelocity.setToZero();
         tempAngularAcceleration.setToZero();
      }
      else
      {
         double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
         orientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);

         if (orientationOffsetTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
         {
            double firstTrajectoryPointTime = orientationOffsetTrajectoryGenerator.getLastWaypointTime();
            PelvisOrientationTrajectoryCommand command = commandQueue.poll();
            numberOfQueuedCommands.decrement();
            initializeOffsetTrajectoryGenerator(command, firstTrajectoryPointTime);
            orientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
         }

         orientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
      }

//      desiredPelvisOrientationOffset.set(tempOrientation);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      if (addPelvisOffsetsBasedOnStep.getBooleanValue())
      {
         computePelvisOffsetWhileWalking();
      }
      else
      {
         desiredPelvisOffsetWhileWalking.setToZero();
      }

      double desiredYaw = tempOrientation.getYaw();
      double desiredPitch = tempOrientation.getPitch();
      double desiredRoll = tempOrientation.getRoll();

      desiredYaw += desiredPelvisOffsetWhileWalking.getYaw().getDoubleValue();
      desiredPitch += desiredPelvisOffsetWhileWalking.getPitch().getDoubleValue();
      desiredRoll += desiredPelvisOffsetWhileWalking.getRoll().getDoubleValue();

      tempOrientation.setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);

      desiredPelvisQuaternionWithOffset.set(tempOrientation);
      desiredPelvisOrientationWithOffset.set(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      desiredPelvisQuaternionWithOffset.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      desiredPelvisAngularAcceleration.getFrameTupleIncludingFrame(tempAngularAcceleration);

      orientationFeedbackControlCommand.set(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
      yoPelvisAngularWeight.get(pelvisAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(pelvisAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);
   }

   public void goToHomeFromCurrentDesired()
   {
      goToHomeFromCurrentDesired(defaultTrajectoryTime);
   }

   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      orientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
      tempOrientation.changeFrame(desiredPelvisFrame);
      tempAngularVelocity.setToZero(desiredPelvisFrame);

      orientationOffsetTrajectoryGenerator.clear();
      orientationOffsetTrajectoryGenerator.switchTrajectoryFrame(desiredPelvisFrame);
      orientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      tempOrientation.setToZero(desiredPelvisFrame);
      orientationOffsetTrajectoryGenerator.appendWaypoint(trajectoryTime, tempOrientation, tempAngularVelocity);
      orientationOffsetTrajectoryGenerator.initialize();

      isTrajectoryStopped.set(false);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
   }

   private final PelvisOrientationTrajectoryCommand tempPelvisOrientationTrajectoryCommand = new PelvisOrientationTrajectoryCommand();

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      tempPelvisOrientationTrajectoryCommand.set(command);
      handlePelvisOrientationTrajectoryCommands(tempPelvisOrientationTrajectoryCommand);
   }

   public void handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command)
   {
      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(command.getCommandId());
         initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
         initializeOffsetTrajectoryGenerator(command, 0.0);
         return;
      case QUEUE:
         boolean success = queuePelvisOrientationTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            setToHoldCurrent(pelvisOrientationTrajectoryGenerator.getCurrentTrajectoryFrame());
         }
         return;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         break;
      }
   }

   private boolean queuePelvisOrientationTrajectoryCommand(PelvisOrientationTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         PrintTools.warn(this, "The very first " + command.getClass().getSimpleName() + " of a series must be " + ExecutionMode.OVERRIDE + ". Aborting motion.");
         return false;
      }

      long previousCommandId = command.getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         PrintTools.warn(this, "Previous command ID mismatch: previous ID from command = " + previousCommandId
               + ", last message ID received by the controller = " + lastCommandId.getLongValue() + ". Aborting motion.");
         return false;
      }

      if (command.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         PrintTools.warn(this, "Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getCommandId());

      return true;
   }

   private void initializeOffsetTrajectoryGenerator(PelvisOrientationTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      command.addTimeOffset(firstTrajectoryPointTime);

      if (command.getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         orientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);
         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         orientationOffsetTrajectoryGenerator.clear();
         orientationOffsetTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }
      else
      {
         orientationOffsetTrajectoryGenerator.clear();
         orientationOffsetTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         orientationOffsetTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      orientationOffsetTrajectoryGenerator.changeFrame(desiredPelvisFrame);
      orientationOffsetTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisOrientationTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = orientationOffsetTrajectoryGenerator.getMaximumNumberOfWaypoints() - orientationOffsetTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisOrientationTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.setPropertiesOnly(command);

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.addTrajectoryPoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(desiredPelvisFrame);
      tempAngularVelocity.setToZero(desiredPelvisFrame);
      orientationOffsetTrajectoryGenerator.clear();
      orientationOffsetTrajectoryGenerator.switchTrajectoryFrame(desiredPelvisFrame);
      orientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      orientationOffsetTrajectoryGenerator.initialize();
   }

   public void setToHoldCurrentInWorldFrame()
   {
      setToHoldCurrent(worldFrame);
   }

   public void setToHoldCurrent(ReferenceFrame trajectoryFrame)
   {
      tempOrientation.setToZero(pelvisFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisQuaternion.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      resetOrientationOffset();
      initialize(trajectoryFrame);
   }

   public void prepareForLocomotion()
   {
      desiredPelvisQuaternionWithOffset.getFrameOrientationIncludingFrame(tempOrientation);
      tempOrientation.changeFrame(initialPelvisOrientation.getReferenceFrame());
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisQuaternion.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      resetOrientationOffset();
      initialize(worldFrame);
   }

   public void setToHoldCurrentDesiredInWorldFrame()
   {
      setToHoldCurrentDesired(worldFrame);
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      setToHoldCurrentDesired(midFeetZUpFrame);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      setToHoldCurrentDesired(ankleZUpFrames.get(supportSide));
   }

   public void setToHoldCurrentDesired(ReferenceFrame desiredTrajectoryFrame)
   {
      desiredPelvisQuaternion.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);

      initialize(desiredTrajectoryFrame);
   }

   /** Go instantly to zero, no smooth interpolation. */
   public void setToZeroInSupportFoot(RobotSide supportSide)
   {
      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(supportSide);
      tempOrientation.setToZero(supportAnkleZUp);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisQuaternion.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize(supportAnkleZUp);
   }

   /** Go instantly to zero, no smooth interpolation. */
   public void setToZeroInMidFeetZUpFrame()
   {
      tempOrientation.setToZero(midFeetZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisQuaternion.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize(midFeetZUpFrame);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      desiredPelvisQuaternion.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      ReferenceFrame otherAnkleZUpFrame = ankleZUpFrames.get(supportSide.getOppositeSide());
      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportSide);

      tempOrientation.setToZero(otherAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawOtherFoot = tempOrientation.getYaw();

      tempOrientation.setToZero(supportAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawOtherFoot, yawSupportFoot);

      finalPelvisOrientation.set(finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(supportAnkleZUpFrame);
   }

   /** Move towards zero smoothly within the given swing time */
   public void moveToZeroInSupportFoot(RobotSide supportSide)
   {
      desiredPelvisQuaternion.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(supportSide);
      tempOrientation.setToZero(supportAnkleZUp);
      tempOrientation.changeFrame(worldFrame);
      finalPelvisOrientation.set(tempOrientation);

      initialize(supportAnkleZUp);
   }

   private final FramePoint upcomingFootstepLocation = new FramePoint();
   private final FrameOrientation upcomingFootstepOrientation = new FrameOrientation();

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      supportSide = upcomingFootstep.getRobotSide().getOppositeSide();
      RigidBodyTransform ankleToSole = transformsFromAnkleToSole.get(upcomingFootstep.getRobotSide());
      upcomingFootstep.getAnkleOrientation(upcomingFootstepOrientation, ankleToSole);
      upcomingFootstep.getAnklePosition(upcomingFootstepLocation, ankleToSole);
   }

   public void setTrajectoryFromFootstep()
   {
      RobotSide upcomingFootstepSide = supportSide.getOppositeSide();

      desiredPelvisQuaternion.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      upcomingFootstepOrientation.changeFrame(worldFrame);
      tempOrientation.setToZero(ankleZUpFrames.get(upcomingFootstepSide.getOppositeSide()));
      tempOrientation.changeFrame(worldFrame);

      // computes the yaw angle as being the average of the stance orientation yaw and the footstep orientation yaw
      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(upcomingFootstepOrientation.getYaw(), tempOrientation.getYaw());

      upcomingFootstepLocation.changeFrame(ankleZUpFrames.get(upcomingFootstepSide.getOppositeSide()));

      // adds additional yaw to account for the angle between the actual feet
      double desiredSwingPelvisYawAngle = computeDesiredSwingPelvisYawAngleFromStep(upcomingFootstepSide, upcomingFootstepLocation, 0.1);
      swingPelvisYaw.set(desiredSwingPelvisYawAngle);

      finalPelvisOrientation.set(finalDesiredPelvisYawAngle + swingPelvisYawScale.getDoubleValue() * desiredSwingPelvisYawAngle, 0.0, 0.0);

      initialize(worldFrame);
   }

   private double computeDesiredSwingPelvisYawAngleFromStep(RobotSide upcomingFootstepSide, FramePoint footLocation, double minimumStepForward)
   {
      double desiredSwingPelvisYawAngle = 0.0;
      if (Math.abs(footLocation.getX()) > minimumStepForward)
      {
         desiredSwingPelvisYawAngle = Math.atan2(footLocation.getY(), footLocation.getX());
         desiredSwingPelvisYawAngle -= upcomingFootstepSide.negateIfRightSide(Math.PI / 2.0);
      }

      return desiredSwingPelvisYawAngle;
   }

   private double computeStepAngle(FramePoint footLocation)
   {
      double stepAngle = 0.0;
      if (Math.abs(footLocation.getX()) > 0.03)
         stepAngle = Math.atan2(footLocation.getX(), footLocation.getY());

      return stepAngle;
   }

   public void initializeStanding()
   {
      isStanding.set(true);
      isInTransfer.set(false);
   }

   public void submitTiming(FootstepTiming timing)
   {
      transferDuration.set(timing.getTransferTime());
      swingDuration.set(timing.getSwingTime());
   }

   private final FramePoint tmpPoint = new FramePoint();
   public void initializeTransfer(RobotSide transferToSide)
   {
      supportSide = transferToSide;

      initialTime = yoTime.getDoubleValue();

      if (!PELVIS_YAW_FROM_FEET_POSITIONS)
      {
         double stepDuration = transferDuration.getDoubleValue() + swingDuration.getDoubleValue();
         this.pelvisYawSineFrequency.set(1.0 / (2.0 * stepDuration));
         this.pelvisYawSineMagnitude.set(transferToSide.negateIfRightSide(Math.toRadians(pelvisYawAngle.getDoubleValue())));
      }
      else
      {
         // compute pelvis transfer magnitude
         tmpPoint.setToZero(ankleZUpFrames.get(supportSide));
         tmpPoint.changeFrame(ankleZUpFrames.get(supportSide.getOppositeSide()));
         double stepAngle = transferToSide.negateIfLeftSide(computeStepAngle(tmpPoint));
         double pelvisYawSineMagnitude = pelvisYawRatio.getDoubleValue() * stepAngle;

         // compute pelvis frequency
         double initialPelvisDesiredYaw = desiredPelvisOffsetWhileWalking.getYaw().getDoubleValue();
         double pelvisYawSineFrequency = 0.0;
         if (pelvisYawSineMagnitude != initialPelvisDesiredYaw)
            pelvisYawSineFrequency = 1.0 / (2.0 * Math.PI * transferDuration.getDoubleValue()) * Math.asin(initialPelvisDesiredYaw / pelvisYawSineMagnitude);

         this.pelvisYawSineMagnitude.set(-pelvisYawSineMagnitude);
         this.pelvisYawSineFrequency.set(pelvisYawSineFrequency);
      }

      isStanding.set(false);
      isInTransfer.set(true);
   }

   public void initializeSwing(RobotSide supportSide)
   {
      this.supportSide = supportSide;

      initialTime = yoTime.getDoubleValue();

      if (!PELVIS_YAW_FROM_FEET_POSITIONS)
      {
         double stepDuration = transferDuration.getDoubleValue() + swingDuration.getDoubleValue();
         this.pelvisYawSineFrequency.set(1.0 / (2.0 * stepDuration));
         this.pelvisYawSineMagnitude.set(supportSide.negateIfRightSide(Math.toRadians(pelvisYawAngle.getDoubleValue())));
      }
      else
      {
         // compute pelvis transfer magnitude
         tmpPoint.setToZero(ankleZUpFrames.get(supportSide));
         tmpPoint.changeFrame(ankleZUpFrames.get(supportSide.getOppositeSide()));
         double stepAngle = supportSide.negateIfRightSide(computeStepAngle(upcomingFootstepLocation));
         double pelvisYawSineMagnitude = pelvisYawRatio.getDoubleValue() * stepAngle;
         this.pelvisYawSineMagnitude.set(pelvisYawSineMagnitude);

         // compute pelvis frequency
         double stepDuration = transferDuration.getDoubleValue() + swingDuration.getDoubleValue();
         this.pelvisYawSineFrequency.set(1.0 / (2.0 * stepDuration));
      }

      isStanding.set(false);
      isInTransfer.set(false);
   }

   private void computePelvisOffsetWhileWalking()
   {
      if (isStanding.getBooleanValue())
      {
         desiredPelvisOffsetWhileWalking.setToZero();
      }
      else
      {
         timeInState.set(yoTime.getDoubleValue() - initialTime);

         if (isInTransfer.getBooleanValue())
         {
            updatePelvisPitchOffsetInTransfer(supportSide);
            updatePelvisYawOffsetInTransfer();
         }
         else
         {
            updatePelvisPitchOffsetInSwing(supportSide);
            updatePelvisYawOffsetInSwing();
         }
      }
   }

   private final FramePoint stanceLine = new FramePoint();
   private final FramePoint trailingStanceLine = new FramePoint();
   private void updatePelvisPitchOffsetInTransfer(RobotSide transferToSide)
   {
      stanceLine.setToZero(pelvisFrame);
      stanceLine.changeFrame(ankleZUpFrames.get(transferToSide));
      double distanceFromSupportFoot = stanceLine.getX();
      double heightFromSupportFoot = stanceLine.getZ();
      double leadingLegAngle = Math.atan2(distanceFromSupportFoot, heightFromSupportFoot);

      trailingStanceLine.setToZero(pelvisFrame);
      trailingStanceLine.changeFrame(ankleZUpFrames.get(transferToSide.getOppositeSide()));
      double distanceFromTrailingFoot = trailingStanceLine.getX();
      double heightFromTrailingFoot = trailingStanceLine.getZ();
      double trailingLegAngle = Math.atan2(distanceFromTrailingFoot, heightFromTrailingFoot);

      double phaseInState = timeInState.getDoubleValue() / transferDuration.getDoubleValue();

      double interpolatedLegAngle = TupleTools.interpolate(trailingLegAngle, leadingLegAngle, phaseInState);

      desiredPelvisOffsetWhileWalking.setPitch(pelvisPitchAngleRatio.getDoubleValue() * interpolatedLegAngle);
   }

   private void updatePelvisPitchOffsetInSwing(RobotSide supportSide)
   {
      stanceLine.setToZero(pelvisFrame);
      stanceLine.changeFrame(ankleZUpFrames.get(supportSide));
      double distanceFromFoot = stanceLine.getX();
      double heightFromFoot = stanceLine.getZ();
      double supportLegAngle = Math.atan2(distanceFromFoot, heightFromFoot);

      desiredPelvisOffsetWhileWalking.setPitch(pelvisPitchAngleRatio.getDoubleValue() * supportLegAngle);
   }
 
   private void updatePelvisYawOffsetInTransfer()
   {
      if (!PELVIS_YAW_FROM_FEET_POSITIONS)
      {
         pelvisYawSineMagnitude.set(supportSide.negateIfRightSide(Math.toRadians(pelvisYawAngle.getDoubleValue())));

         double timeInStep = timeInState.getDoubleValue();
         double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInStep * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
         desiredPelvisOffsetWhileWalking.setYaw(yaw);
      }
      else
      {
         double timeInSine = timeInState.getDoubleValue() - transferDuration.getDoubleValue();
         double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInSine * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
         desiredPelvisOffsetWhileWalking.setYaw(yaw);
      }
   }

   private void updatePelvisYawOffsetInSwing()
   {
      if (!PELVIS_YAW_FROM_FEET_POSITIONS)
      {
         pelvisYawSineMagnitude.set(supportSide.negateIfRightSide(Math.toRadians(pelvisYawAngle.getDoubleValue())));

         double timeInStep = timeInState.getDoubleValue() + transferDuration.getDoubleValue();
         double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInStep * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
         desiredPelvisOffsetWhileWalking.setYaw(yaw);
      }
      else
      {
         double timeInStep = timeInState.getDoubleValue();
         double yaw = pelvisYawSineMagnitude.getDoubleValue() * Math.sin(timeInStep * pelvisYawSineFrequency.getDoubleValue() * 2.0 * Math.PI);
         desiredPelvisOffsetWhileWalking.setYaw(yaw);
      }
   }


   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}
