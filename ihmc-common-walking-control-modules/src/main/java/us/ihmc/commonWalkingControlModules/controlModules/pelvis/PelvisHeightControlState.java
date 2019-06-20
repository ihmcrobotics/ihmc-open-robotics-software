package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyPositionController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.CommandConversionTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.AbstractPDController;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricPID3DGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisHeightControlState implements PelvisAndCenterOfMassHeightControlState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   /**
    * We take the spatialFeedback command from the RigidBodyTaskspaceControlState and pack it into a
    * point feedback command and set the selection matrix to Z only
    **/
   private final SelectionMatrix3D temp3DSelection = new SelectionMatrix3D();

   /**
    * When we handle the PelvisTrajectoryCommand we pull out the z component and pack it into another
    * PelvisTrajectoryCommand
    **/
   private final EuclideanTrajectoryControllerCommand euclideanCommand = new EuclideanTrajectoryControllerCommand();

   /** handles the trajectory and the queuing **/
   private final RigidBodyPositionController positionController;

   private final RigidBodyBasics pelvis;
   private final MovingReferenceFrame pelvisFrame;
   private final ReferenceFrame baseFrame;

   private final DoubleProvider defaultHeight;
   private final DoubleProvider minHeight;

   private final DoubleProvider offset;
   private final DoubleProvider offsetTrajectoryTime;
   private double previousOffset = 0.0;

   private final FramePoint3D tempPosition = new FramePoint3D();

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

   private final FramePoint3D statusDesiredPosition = new FramePoint3D();
   private final FramePoint3D statusActualPosition = new FramePoint3D();
   private final TaskspaceTrajectoryStatusMessageHelper statusHelper = new TaskspaceTrajectoryStatusMessageHelper("pelvisHeight");

   public PelvisHeightControlState(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();

      pelvis = fullRobotModel.getPelvis();
      pelvisFrame = referenceFrames.getPelvisFrame();
      // The base frame could be switched to the mid foot frame at some point.
      baseFrame = elevator.getBodyFixedFrame();

      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      DoubleProvider proportionalGain = () -> symmetric3DGains.getProportionalGains()[2];
      DoubleProvider derivativeGain = () -> symmetric3DGains.getDerivativeGains()[2];
      linearMomentumZPDController = AbstractPDController.createPDController("pelvisHeightControlState_linearMomentumZPDController", proportionalGain,
                                                                            derivativeGain, () -> 0.0, registry);
      yoControlFrame = new YoSE3OffsetFrame(pelvis.getName() + "HeightBodyFixedControlFrame", pelvis.getBodyFixedFrame(), registry);

      positionController = new RigidBodyPositionController(pelvis, elevator, elevator, pelvisFrame, baseFrame, yoTime, registry, graphicsListRegistry);

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

   @Override
   public void initialize()
   {
   }

   public void setGains(PIDGainsReadOnly gains)
   {
      symmetric3DGains.setGains(gains);
      positionController.setGains(symmetric3DGains);
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

      command.clear();
      trajectoryPoint.setToZero();

      trajectoryPoint.setZ(adjustedDesiredHeight);
      command.addTrajectoryPoint(time, trajectoryPoint, zeroVelocity);

      positionController.handleTrajectoryCommand(command);
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
    * 
    * @param linearWeight
    */
   public void setWeights(Vector3DReadOnly linearWeight)
   {
      positionController.setWeights(linearWeight);
   }

   @Override
   public void doAction(double timeInState)
   {
      // This is to avoid a variable changed listener that would fire when using the SCS playback.
      if (offset.getValue() != previousOffset)
      {
         double desiredZInworld = positionController.getFeedbackControlCommand().getReferencePosition().getZ();
         double currentDesired = desiredZInworld + adjustmentAmount.getValue();
         goToHeight(currentDesired - previousOffset, offsetTrajectoryTime.getValue());
      }
      previousOffset = offset.getValue();

      positionController.doAction(Double.NaN);
      statusHelper.updateWithTimeInTrajectory(positionController.getTimeInTrajectory());
   }

   public boolean handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();

      if (positionController.handleTrajectoryCommand(euclideanTrajectory))
      {
         euclideanTrajectory.setSequenceId(command.getSequenceId());
         statusHelper.registerNewTrajectory(euclideanTrajectory);
         return true;
      }

      initializeDesiredHeightToCurrent();
      return false;
   }

   /**
    * check that the command is valid and queue the trajectory
    * 
    * @param command
    * @param initialPose the initial pelvis position
    * @return whether the command passed validation and was queued
    */
   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      CommandConversionTools.convertToEuclidean(command.getSE3Trajectory(), euclideanCommand);

      ReferenceFrame selectionFrame = euclideanCommand.getSelectionMatrix().getSelectionFrame();
      if (selectionFrame != null && !selectionFrame.isZupFrame())
      {
         LogTools.warn("Pelvis linear selection matrix must be z-up!");
         return false;
      }
      euclideanCommand.getSelectionMatrix().selectXAxis(false);
      euclideanCommand.getSelectionMatrix().selectYAxis(false);
      euclideanCommand.getSelectionMatrix().setSelectionFrame(ReferenceFrame.getWorldFrame());

      ReferenceFrame weightFrame = euclideanCommand.getWeightMatrix().getWeightFrame();
      if (weightFrame != null && !weightFrame.isZupFrame())
      {
         LogTools.warn("Pelvis linear weight matrix must be z-up!");
         return false;
      }
      euclideanCommand.getWeightMatrix().setXAxisWeight(0.0);
      euclideanCommand.getWeightMatrix().setYAxisWeight(0.0);
      euclideanCommand.getWeightMatrix().setWeightFrame(ReferenceFrame.getWorldFrame());

      if (positionController.handleTrajectoryCommand(euclideanCommand))
      {
         euclideanCommand.setSequenceId(command.getSequenceId());
         statusHelper.registerNewTrajectory(euclideanCommand);
         return true;
      }

      initializeDesiredHeightToCurrent();
      return false;
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      positionController.holdCurrent();
   }

   @Override
   public void goHome(double trajectoryTime)
   {
      tempPosition.setToZero(baseFrame);
      tempPosition.setZ(defaultHeight.getValue());
      positionController.goToPositionFromCurrent(tempPosition, trajectoryTime);
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
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final Twist twist = new Twist();

   /**
    * returns the point feedback command for the z height of the pelvis
    */
   @Override
   public PointFeedbackControlCommand getFeedbackControlCommand()
   {
      PointFeedbackControlCommand feedbackCommand = positionController.getFeedbackControlCommand();

      // TODO: for some reason this is needed to avoid robot-blow up. It should be selected out by the selection matrix!
      feedForwardLinearAcceleration.setIncludingFrame(feedbackCommand.getReferenceLinearAcceleration());
      feedForwardLinearAcceleration.setX(0.0);
      feedForwardLinearAcceleration.setY(0.0);
      feedbackCommand.getReferenceLinearAcceleration().set(feedForwardLinearAcceleration);

      temp3DSelection.clearSelection();
      temp3DSelection.setSelectionFrame(ReferenceFrame.getWorldFrame());
      temp3DSelection.selectZAxis(true);
      feedbackCommand.setSelectionMatrix(temp3DSelection);

      return feedbackCommand;
   }

   @Override
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
                                                     FeetManager feetManager)
   {
      PointFeedbackControlCommand feedbackCommand = positionController.getFeedbackControlCommand();

      controlPosition.setIncludingFrame(feedbackCommand.getBodyFixedPointToControl());
      controlPosition.changeFrame(pelvis.getBodyFixedFrame());
      yoControlFrame.setOffsetToParentToTranslationOnly(controlPosition);
      yoControlFrame.getTwistRelativeToOther(baseFrame, twist);
      currentLinearVelocity.setIncludingFrame(twist.getLinearPart());

      currentLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      controlPosition.changeFrame(ReferenceFrame.getWorldFrame());

      currentPelvisHeightInWorld.set(controlPosition.getZ());
      desiredPelvisHeightInWorld.set(feedbackCommand.getReferencePosition().getZ());
      currentPelvisVelocityInWorld.set(currentLinearVelocity.getZ());
      desiredPelvisVelocityInWorld.set(feedbackCommand.getReferenceLinearVelocity().getZ());

      return linearMomentumZPDController.compute(currentPelvisHeightInWorld.getValue(), desiredPelvisHeightInWorld.getValue(),
                                                 currentPelvisVelocityInWorld.getValue(), desiredPelvisVelocityInWorld.getValue());
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      statusDesiredPosition.setIncludingFrame(positionController.getFeedbackControlCommand().getReferencePosition());
      statusDesiredPosition.setX(Double.NaN);
      statusDesiredPosition.setY(Double.NaN);
      statusActualPosition.setIncludingFrame(controlPosition);
      statusActualPosition.setX(Double.NaN);
      statusActualPosition.setY(Double.NaN);

      return statusHelper.pollStatusMessage(statusDesiredPosition, statusActualPosition);
   }
}
