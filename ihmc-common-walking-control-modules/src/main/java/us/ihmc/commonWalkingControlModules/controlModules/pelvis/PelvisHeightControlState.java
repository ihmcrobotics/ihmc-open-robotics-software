package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.AbstractPDController;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricPID3DGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisHeightControlState implements PelvisAndCenterOfMassHeightControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   /** We take the spatialFeedback command from the RigidBodyTaskspaceControlState and pack it into a point feedback command and set the selection matrix to Z only**/
   private final PointFeedbackControlCommand pointFeedbackCommand = new PointFeedbackControlCommand();
   private final SelectionMatrix6D linearZSelectionMatrix = new SelectionMatrix6D();
   private final SelectionMatrix3D temp3DSelection = new SelectionMatrix3D();
   private final WeightMatrix6D linearZWeightMatrix = new WeightMatrix6D();

   /** When we handle the PelvisTrajectoryCommand we pull out the z component and pack it into another PelvisTrajectoryCommand**/
   private final PelvisTrajectoryCommand tempPelvisTrajectoryCommand = new PelvisTrajectoryCommand();

   /** handles the trajectory and the queuing**/
   private final RigidBodyTaskspaceControlState taskspaceControlState;

   private final RigidBody pelvis;
   private final MovingReferenceFrame pelvisFrame;
   private final ReferenceFrame baseFrame;

   private final DoubleProvider defaultHeight;
   private final DoubleProvider minHeight;

   private final DoubleProvider offset;
   private final DoubleProvider offsetTrajectoryTime;
   private double previousOffset = 0.0;

   private final FramePose3D tempPose = new FramePose3D();
   private final Point3D tempPoint = new Point3D();
   private final RigidBodyTransform controlFrame = new RigidBodyTransform();

   private final AbstractPDController linearMomentumZPDController;
   private final YoSE3OffsetFrame yoControlFrame;

   private final YoDouble currentPelvisHeightInWorld;
   private final YoDouble desiredPelvisHeightInWorld;
   private final YoDouble desiredPelvisVelocityInWorld;
   private final YoDouble currentPelvisVelocityInWorld;

   private final SymmetricPID3DGains symmetric3DGains = new SymmetricPID3DGains();

   // Used for singularity avoidance to make sure the distance between ankle and pelvis never exceeds a user defined distance.
   private final SideDependentList<MovingReferenceFrame> ankleFrames;
   private final DoubleProvider maxDistanceAnklePelvis;
   private final YoBoolean adjustedDesiredForSingularity;
   private final YoDouble adjustmentAmount;
   private final FramePoint3D pelvisPosition = new FramePoint3D();
   private final FramePoint3D anklePosition = new FramePoint3D();
   private final Vector3D ankleToPelvis = new Vector3D();
   private final Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);

   private RobotSide swingSide = null;
   private double toeOffHeight = 0.0;

   private final EuclideanTrajectoryControllerCommand command = new EuclideanTrajectoryControllerCommand();
   private final Vector3D zeroVelocity = new Vector3D();
   private final Point3D trajectoryPoint = new Point3D();

   public PelvisHeightControlState(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBody elevator = fullRobotModel.getElevator();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();

      pelvis = fullRobotModel.getPelvis();
      pelvisFrame = referenceFrames.getPelvisFrame();
      // The base frame could be switched to the mid foot frame at some point.
      baseFrame = elevator.getBodyFixedFrame();

      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      DoubleProvider proportionalGain = () -> symmetric3DGains.getProportionalGains()[2];
      DoubleProvider derivativeGain = () -> symmetric3DGains.getDerivativeGains()[2];
      linearMomentumZPDController = AbstractPDController.createPDController("pelvisHeightControlState_linearMomentumZPDController", proportionalGain,
                                                                            derivativeGain, () -> 0.0, registry);
      yoControlFrame = new YoSE3OffsetFrame(pelvis.getName() + "HeightBodyFixedControlFrame", pelvis.getBodyFixedFrame(), registry);

      taskspaceControlState = new RigidBodyTaskspaceControlState("Height", pelvis, elevator, elevator, trajectoryFrames, pelvisFrame, baseFrame, yoTime, null,
                                                                 graphicsListRegistry, registry);

      defaultHeight = new DoubleParameter(getClass().getSimpleName() + "DefaultHeight", registry);
      minHeight = new DoubleParameter(getClass().getSimpleName() + "MinHeight", registry, 0.0);
      maxDistanceAnklePelvis = new DoubleParameter(getClass().getSimpleName() + "MaxDistanceAnklePelvis", registry, Double.POSITIVE_INFINITY);
      offset = new DoubleParameter(getClass().getSimpleName() + "Offset", registry, 0.0);
      offsetTrajectoryTime = new DoubleParameter(getClass().getSimpleName() + "OffsetTrajectoryTime", registry, 0.5);

      ankleFrames = controllerToolbox.getReferenceFrames().getAnkleZUpReferenceFrames();
      adjustedDesiredForSingularity = new YoBoolean("AdjustedDesiredForSingularity", registry);
      adjustmentAmount = new YoDouble("AdjustmentAmount", registry);

      currentPelvisHeightInWorld = new YoDouble("currentPelvisHeightInWorld", registry);
      desiredPelvisHeightInWorld = new YoDouble("desiredPelvisHeightInWorld", registry);
      desiredPelvisVelocityInWorld = new YoDouble("desiredPelvisVelocityInWorld", registry);
      currentPelvisVelocityInWorld = new YoDouble("currentPelvisVelocityInWorld", registry);

      parentRegistry.addChild(registry);
   }

   public void setGains(PIDGainsReadOnly gains)
   {
      symmetric3DGains.setGains(gains);
      taskspaceControlState.setGains(null, symmetric3DGains);
   }

   public void step(Point3DReadOnly stanceFootPosition, Point3DReadOnly touchdownPosition, double swingTime, RobotSide swingSide, double toeOffHeight)
   {
      this.swingSide = swingSide;
      this.toeOffHeight = toeOffHeight;

      double r = defaultHeight.getValue();

      // In a frame rotated up so the a axis matches the incline of the step the waypoing is here:
      double x_incl = 0.5 * stanceFootPosition.distance(touchdownPosition);
      double z_incl = Math.sqrt(r * r - x_incl * x_incl);

      // The angle of rotation up is:
      double stepLength = stanceFootPosition.distanceXY(touchdownPosition);
      double zTouchdown = touchdownPosition.getZ();
      double stepHeight = zTouchdown - stanceFootPosition.getZ();
      double inclination = Math.atan2(stepHeight, stepLength);

      // Rotate the coordinates:
      double x = Math.cos(inclination) * x_incl - Math.sin(inclination) * z_incl;
      double z = Math.sin(inclination) * x_incl + Math.cos(inclination) * z_incl + toeOffHeight;
      MathTools.clamp(z, minHeight.getValue(), Double.POSITIVE_INFINITY);

      // Compute the distance into the step that the low point is reached:
      double alpha = x / stanceFootPosition.distanceXY(touchdownPosition);
      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      // Compute the mid step waypoint:
      double zInWorld = stanceFootPosition.getZ() + z;
      zInWorld = MathTools.clamp(zInWorld, zTouchdown + minHeight.getValue(), Double.POSITIVE_INFINITY);
      goToHeight(zInWorld, swingTime);
   }

   public void transfer(Point3DReadOnly transferPosition, double transferTime, RobotSide swingSide, double toeOffHeight)
   {
      this.swingSide = swingSide;
      this.toeOffHeight = toeOffHeight;

      // Compute the waypoint above the footstep to transfer to:
      double desiredHeight = transferPosition.getZ() + defaultHeight.getValue();
      goToHeight(desiredHeight, transferTime);
   }

   private void goToHeight(double desiredHeight, double time)
   {
      double offsetDesiredHeight = desiredHeight + offset.getValue();
      double adjustedDesiredHeight = avoidSingularities(offsetDesiredHeight);

      trajectoryPoint.setToZero();
      trajectoryPoint.setZ(adjustedDesiredHeight);

      command.clear();
      command.addTrajectoryPoint(time, trajectoryPoint, zeroVelocity);

      taskspaceControlState.setDefaultControlFrame();
      taskspaceControlState.getDesiredPose(tempPose);
      taskspaceControlState.handleEuclideanTrajectoryCommand(command, tempPose);
   }

   private double avoidSingularities(double height)
   {
      pelvisPosition.setToZero(pelvisFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPosition.setZ(height);

      boolean heightWasAdjusted = false;
      double adjustment = 0.0;
      for (int sideIdx = 0; sideIdx < RobotSide.values.length; sideIdx++)
      {
         RobotSide side = RobotSide.values[sideIdx];
         anklePosition.setToZero(ankleFrames.get(side));
         anklePosition.changeFrame(ReferenceFrame.getWorldFrame());

         double maxDistance = maxDistanceAnklePelvis.getValue();
         if (side == swingSide)
         {
            maxDistance = maxDistance + toeOffHeight;
         }

         double distanceAnkleDesiredPelvis = anklePosition.distance(pelvisPosition);
         double adjustmentAlongLeg = distanceAnkleDesiredPelvis - maxDistance;

         if (adjustmentAlongLeg > 0.0)
         {
            ankleToPelvis.sub(pelvisPosition, anklePosition);
            double adjustmentAlongZ = adjustmentAlongLeg / Math.abs(Math.cos(ankleToPelvis.dot(zAxis)));
            adjustment = Math.max(adjustment, adjustmentAlongZ);
            heightWasAdjusted = true;
         }
      }

      adjustmentAmount.set(adjustment);
      adjustedDesiredForSingularity.set(heightWasAdjusted);
      return pelvisPosition.getZ() - adjustment;
   }

   /**
    * set the qp weights for the taskspace linear z command
    * @param linearWeight
    */
   public void setWeights(Vector3DReadOnly linearWeight)
   {
      taskspaceControlState.setWeights(null, linearWeight);
   }

   @Override
   public void doAction(double timeInState)
   {
      // This is to avoid a variable changed listener that would fire when using the SCS playback.
      if (offset.getValue() != previousOffset)
      {
         taskspaceControlState.getDesiredPose(tempPose);
         tempPose.changeFrame(ReferenceFrame.getWorldFrame());
         double currentDesired = tempPose.getZ() + adjustmentAmount.getValue();
         goToHeight(currentDesired - previousOffset, offsetTrajectoryTime.getValue());
      }
      previousOffset = offset.getValue();

      taskspaceControlState.doAction(Double.NaN);
   }

   public boolean handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command, FramePose3D initialPose)
   {
      if (command.getEuclideanTrajectory().useCustomControlFrame())
      {
         tempPelvisTrajectoryCommand.getSE3Trajectory().getControlFramePose(controlFrame);
         taskspaceControlState.setControlFramePose(controlFrame);
      }
      else
      {
         taskspaceControlState.setDefaultControlFrame();
      }

      // Convert the initial point to be consistent with the control frame
      ReferenceFrame controlFrame = taskspaceControlState.getControlFrame();
      tempPose.setToZero(pelvisFrame);
      tempPose.changeFrame(controlFrame);
      tempPoint.set(tempPose.getPosition());

      initialPose.prependTranslation(tempPoint);

      return taskspaceControlState.handleEuclideanTrajectoryCommand(command.getEuclideanTrajectory(), initialPose);
   }

   /**
    * check that the command is valid and queue the trajectory
    * @param command
    * @param initialPose the initial pelvis position
    * @return whether the command passed validation and was queued
    */
   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command, FramePose3D initialPose)
   {
      // We have to remove the orientation and xy components of the command, and adjust the selection matrix;
      // We do this to break up the pelvis control, it reduces the complexity of each manager at the expense of these little hacks.
      tempPelvisTrajectoryCommand.set(command);

      //set the selection matrix to z only
      SelectionMatrix6D commandSelectionMatrix = tempPelvisTrajectoryCommand.getSE3Trajectory().getSelectionMatrix();
      if (commandSelectionMatrix != null)
      {
         linearZSelectionMatrix.set(commandSelectionMatrix);
         ReferenceFrame linearSelectionFrame = linearZSelectionMatrix.getLinearSelectionFrame();
         if (linearSelectionFrame != null && !linearSelectionFrame.isZupFrame())
         {
            PrintTools.warn("Selection Matrix Linear Frame was not Z up, PelvisTrajectoryCommand can only handle Selection matrix linear components with Z up frames.");
            return false;
         }
      }
      else
      {
         linearZSelectionMatrix.clearLinearSelection();
      }

      linearZSelectionMatrix.clearAngularSelection();
      linearZSelectionMatrix.setLinearAxisSelection(false, false, true);
      linearZSelectionMatrix.setSelectionFrame(ReferenceFrame.getWorldFrame());
      tempPelvisTrajectoryCommand.getSE3Trajectory().setSelectionMatrix(linearZSelectionMatrix);

      //set the weight matrix to z only
      WeightMatrix6D commanedWeightMatrix = tempPelvisTrajectoryCommand.getSE3Trajectory().getWeightMatrix();
      if (commanedWeightMatrix != null)
      {
         linearZWeightMatrix.set(commanedWeightMatrix);
         ReferenceFrame linearWeightFrame = linearZWeightMatrix.getLinearWeightFrame();
         if (linearWeightFrame != null && !linearWeightFrame.isZupFrame())
         {
            PrintTools.warn("Weight Matrix Linear Frame was not Z up, PelvisTrajectoryCommand can only handle weight matrix linear components with Z up frames.");
            return false;
         }
      }
      else
      {
         linearZWeightMatrix.clearLinearWeights();
      }
      linearZWeightMatrix.clearAngularWeights();
      WeightMatrix3D weightLinearPart = linearZWeightMatrix.getLinearPart();
      linearZWeightMatrix.setLinearWeights(0.0, 0.0, weightLinearPart.getZAxisWeight());
      linearZWeightMatrix.setWeightFrame(ReferenceFrame.getWorldFrame());
      tempPelvisTrajectoryCommand.getSE3Trajectory().setWeightMatrix(linearZWeightMatrix);

      if (command.getSE3Trajectory().useCustomControlFrame())
      {
         tempPelvisTrajectoryCommand.getSE3Trajectory().getControlFramePose(controlFrame);
         taskspaceControlState.setControlFramePose(controlFrame);
      }
      else
      {
         taskspaceControlState.setDefaultControlFrame();
      }

      // Convert the initial point to be consistent with the control frame
      ReferenceFrame controlFrame = taskspaceControlState.getControlFrame();
      tempPose.setToZero(pelvisFrame);
      tempPose.changeFrame(controlFrame);
      tempPoint.set(tempPose.getPosition());

      initialPose.prependTranslation(tempPoint);

      if (!taskspaceControlState.handlePoseTrajectoryCommand(tempPelvisTrajectoryCommand.getSE3Trajectory(), initialPose))
      {
         taskspaceControlState.clear();
         return false;
      }
      return true;
   }

   /**
    * returns the control frame which is fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
    * @return
    */
   public ReferenceFrame getControlFrame()
   {
      return taskspaceControlState.getControlFrame();
   }

   /**
    * Packs positionToPack with the current desired height, The parameter's frame will be set to the trajectory frame
    */
   @Override
   public void getCurrentDesiredHeightOfDefaultControlFrame(FramePoint3D positionToPack)
   {
      taskspaceControlState.getDesiredPose(tempPose);
      positionToPack.setIncludingFrame(tempPose.getPosition());

      ReferenceFrame controlFrame = taskspaceControlState.getControlFrame();
      tempPose.setToZero(controlFrame);
      tempPose.changeFrame(pelvisFrame);
      tempPoint.set(tempPose.getPosition());

      positionToPack.add(tempPoint);
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      taskspaceControlState.holdCurrent();
   }

   @Override
   public void goHome(double trajectoryTime)
   {
      tempPose.setToZero(baseFrame);
      tempPose.setZ(defaultHeight.getValue());
      taskspaceControlState.setDefaultControlFrame();
      taskspaceControlState.goToPoseFromCurrent(tempPose, trajectoryTime);
   }

   /**
    * If the command says to stop then set the desired to the actual
    */
   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (command.isStopAllTrajectory())
      {
         initializeDesiredHeightToCurrent();
      }
   }

   private final FramePoint3D controlPosition = new FramePoint3D();
   private final FrameQuaternion controlOrientation = new FrameQuaternion();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final Twist twist = new Twist();

   /**
    * returns the point feedback command for the z height of the pelvis
    */
   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      //We have to do some nasty copying, because the taskspaceControlState returns a spatial feedback command, but the controller core doesn't like
      //when you send two overlapping commands (pelvis orientation uses orientation feedback comand)
      SpatialFeedbackControlCommand spatialFeedbackControlCommand = taskspaceControlState.getSpatialFeedbackControlCommand();

      SpatialAccelerationCommand spcatialAccelerationCommand = spatialFeedbackControlCommand.getSpatialAccelerationCommand();
      pointFeedbackCommand.getSpatialAccelerationCommand().set(spcatialAccelerationCommand);
      pointFeedbackCommand.setControlBaseFrame(spatialFeedbackControlCommand.getControlBaseFrame());
      pointFeedbackCommand.set(spatialFeedbackControlCommand.getBase(), spatialFeedbackControlCommand.getEndEffector());
      spatialFeedbackControlCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity);
      spatialFeedbackControlCommand.getFeedForwardLinearActionIncludingFrame(feedForwardLinearAcceleration);

      // TODO: for some reason this is needed to avoid robot-blow up. It should be selected out by the selection matrix!
      feedForwardLinearAcceleration.setX(0.0);
      feedForwardLinearAcceleration.setY(0.0);

      pointFeedbackCommand.set(desiredPosition, desiredLinearVelocity);
      pointFeedbackCommand.setFeedForwardAction(feedForwardLinearAcceleration);
      pointFeedbackCommand.setControlBaseFrame(spatialFeedbackControlCommand.getControlBaseFrame());
      pointFeedbackCommand.setGains(spatialFeedbackControlCommand.getGains().getPositionGains());
      pointFeedbackCommand.setGainsFrame(baseFrame);
      spatialFeedbackControlCommand.getControlFramePoseIncludingFrame(controlPosition, controlOrientation);
      pointFeedbackCommand.setBodyFixedPointToControl(controlPosition);

      temp3DSelection.clearSelection();
      temp3DSelection.selectZAxis(true);
      pointFeedbackCommand.setSelectionMatrix(temp3DSelection);

      return pointFeedbackCommand;
   }

   @Override
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
                                                     FeetManager feetManager)
   {
      SpatialFeedbackControlCommand spatialFeedbackControlCommand = taskspaceControlState.getSpatialFeedbackControlCommand();
      spatialFeedbackControlCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity);
      spatialFeedbackControlCommand.getFeedForwardLinearActionIncludingFrame(feedForwardLinearAcceleration);
      spatialFeedbackControlCommand.getControlFramePoseIncludingFrame(controlPosition, controlOrientation);
      controlPosition.changeFrame(pelvis.getBodyFixedFrame());

      yoControlFrame.setOffsetToParentToTranslationOnly(controlPosition);
      yoControlFrame.getTwistRelativeToOther(baseFrame, twist);
      twist.getLinearPart(currentLinearVelocity);
      currentLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

      controlPosition.changeFrame(ReferenceFrame.getWorldFrame());

      currentPelvisHeightInWorld.set(controlPosition.getZ());
      desiredPelvisHeightInWorld.set(desiredPosition.getZ());
      desiredPelvisVelocityInWorld.set(desiredLinearVelocity.getZ());
      currentPelvisVelocityInWorld.set(currentLinearVelocity.getZ());

      double acceleration = linearMomentumZPDController.compute(controlPosition.getZ(), desiredPosition.getZ(), currentLinearVelocity.getZ(),
                                                                desiredLinearVelocity.getZ());
      return acceleration;
   }
}
