package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class RigidBodyTaskspaceControlState extends RigidBodyControlState
{
   public static final int maxPoints = 10000;
   public static final int maxPointsInGenerator = 5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final PointFeedbackControlCommand pointFeedbackControlCommand;
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand;
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private PID3DGainsReadOnly orientationGains = null;
   private PID3DGainsReadOnly positionGains = null;

   private final YoBoolean usingWeightFromMessage;
   private final WeightMatrix6D messageWeightMatrix = new WeightMatrix6D();
   private final YoLong messageAngularWeightFrame;
   private final YoLong messageLinearWeightFrame;
   private final YoFrameVector3D messageAngularWeight;
   private final YoFrameVector3D messageLinearWeight;
   private final WeightMatrix6D defaultWeightMatrix = new WeightMatrix6D();
   private Vector3DReadOnly defaultAngularWeight = null;
   private Vector3DReadOnly defaultLinearWeight = null;
   private final YoFrameVector3D currentAngularWeight;
   private final YoFrameVector3D currentLinearWeight;

   private final YoBoolean trackingOrientation;
   private final YoBoolean trackingPosition;

   private final YoBoolean hasOrientationGains;
   private final YoBoolean hasAngularWeight;
   private final YoBoolean hasPositionGains;
   private final YoBoolean hasLinearWeight;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePoint3D desiredPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D(worldFrame);
   private final FrameQuaternion desiredOrientation = new FrameQuaternion(worldFrame);
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D feedForwardAngularAcceleration = new FrameVector3D(worldFrame);

   private final RecyclingArrayDeque<FrameSE3TrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(maxPoints, FrameSE3TrajectoryPoint.class,
                                                                                                     FrameSE3TrajectoryPoint::set);
   private final FrameSE3TrajectoryPoint lastPointAdded = new FrameSE3TrajectoryPoint();

   private final FramePose3D initialPose = new FramePose3D();

   private final ReferenceFrame baseFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame defaultControlFrame;
   private final PoseReferenceFrame controlFrame;
   private ReferenceFrame trajectoryFrame;

   private final FramePose3D controlFramePose = new FramePose3D();

   private final FramePoint3D controlPoint = new FramePoint3D();
   private final YoFramePoint3D yoControlPoint;
   private final FrameQuaternion controlOrientation = new FrameQuaternion();
   private final YoFrameQuaternion yoControlOrientation;
   private final FramePoint3D desiredPoint = new FramePoint3D();
   private final YoFramePoint3D yoDesiredPoint;

   private final YoBoolean hybridModeActive;
   private final RigidBodyJointControlHelper jointControlHelper;

   private final BooleanParameter useBaseFrameForControl = new BooleanParameter("UseBaseFrameForControl", registry, false);
   private final boolean enablePositionTracking;
   private final boolean enableOrientationTracking;

   public RigidBodyTaskspaceControlState(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
                                         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame,
                                         boolean enablePositionTracking, boolean enableOrientationTracking, YoDouble yoTime,
                                         RigidBodyJointControlHelper jointControlHelper, YoGraphicsListRegistry graphicsListRegistry,
                                         YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);
      this.enablePositionTracking = enablePositionTracking;
      this.enableOrientationTracking = enableOrientationTracking;
      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "Taskspace";

      this.baseFrame = baseFrame;
      this.trajectoryFrame = baseFrame;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.controlFrame = new PoseReferenceFrame(prefix + "ControlFrame", bodyFrame);

      trackingOrientation = enableOrientationTracking ? new YoBoolean(prefix + "TrackingOrientation", registry) : null;
      trackingPosition = enablePositionTracking ? new YoBoolean(prefix + "TrackingPosition", registry) : null;

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      spatialFeedbackControlCommand.set(elevator, bodyToControl);
      spatialFeedbackControlCommand.setPrimaryBase(baseBody);
      spatialFeedbackControlCommand.setSelectionMatrixToIdentity();

      if (!enableOrientationTracking)
      {
         pointFeedbackControlCommand = new PointFeedbackControlCommand();
         pointFeedbackControlCommand.set(elevator, bodyToControl);
         pointFeedbackControlCommand.setPrimaryBase(baseBody);
      }
      else
      {
         pointFeedbackControlCommand = null;
      }

      if (!enablePositionTracking)
      {
         orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
         orientationFeedbackControlCommand.set(elevator, bodyToControl);
         orientationFeedbackControlCommand.setPrimaryBase(baseBody);
      }
      else
      {
         orientationFeedbackControlCommand = null;
      }

      defaultControlFrame = controlFrame;
      setControlFrame(defaultControlFrame);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      messageAngularWeight = enableOrientationTracking ? new YoFrameVector3D(prefix + "MessageAngularWeight", null, registry) : null;
      messageLinearWeight = enablePositionTracking ? new YoFrameVector3D(prefix + "MessageLinearWeight", null, registry) : null;
      messageAngularWeightFrame = enableOrientationTracking ? new YoLong(prefix + "MessageAngularReferenceFrame", registry) : null;
      messageLinearWeightFrame = enablePositionTracking ? new YoLong(prefix + "MessageLinearReferenceFrame", registry) : null;

      currentAngularWeight = enableOrientationTracking ? new YoFrameVector3D(prefix + "CurrentAngularWeight", null, registry) : null;
      currentLinearWeight = enablePositionTracking ? new YoFrameVector3D(prefix + "CurrentLinearWeight", null, registry) : null;

      yoControlPoint = new YoFramePoint3D(prefix + "ControlPoint", worldFrame, registry);
      yoControlOrientation = new YoFrameQuaternion(prefix + "ControlOrientation", worldFrame, registry);
      yoDesiredPoint = enablePositionTracking ? new YoFramePoint3D(prefix + "DesiredPoint", worldFrame, registry) : null;

      positionTrajectoryGenerator = enablePositionTracking
            ? new MultipleWaypointsPositionTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry)
            : null;
      orientationTrajectoryGenerator = enableOrientationTracking
            ? new MultipleWaypointsOrientationTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry)
            : null;

      if (trajectoryFrames != null)
      {
         for (ReferenceFrame frameToRegister : trajectoryFrames)
         {
            if (enablePositionTracking)
               positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
            if (enableOrientationTracking)
               orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         }
      }

      if (enablePositionTracking)
         positionTrajectoryGenerator.registerNewTrajectoryFrame(baseFrame);
      if (enableOrientationTracking)
         orientationTrajectoryGenerator.registerNewTrajectoryFrame(baseFrame);

      hasOrientationGains = enableOrientationTracking ? new YoBoolean(prefix + "HasOrientaionGains", registry) : null;
      hasAngularWeight = enableOrientationTracking ? new YoBoolean(prefix + "HasAngularWeights", registry) : null;
      hasPositionGains = enablePositionTracking ? new YoBoolean(prefix + "HasPositionGains", registry) : null;
      hasLinearWeight = enablePositionTracking ? new YoBoolean(prefix + "HasLinearWeights", registry) : null;

      pointQueue.clear();

      this.jointControlHelper = jointControlHelper;
      hybridModeActive = new YoBoolean(prefix + "HybridModeActive", registry);

      defaultWeightMatrix.setAngularWeights(0.0, 0.0, 0.0);
      defaultWeightMatrix.setLinearWeights(0.0, 0.0, 0.0);

      setupViz(graphicsListRegistry, bodyName);
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry, String bodyName)
   {
      if (graphicsListRegistry == null)
         return;

      String listName = getClass().getSimpleName();

      YoGraphicCoordinateSystem controlFrame = new YoGraphicCoordinateSystem(bodyName + "ControlFrame", yoControlPoint, yoControlOrientation, 0.05);
      graphicsListRegistry.registerYoGraphic(listName, controlFrame);
      graphics.add(controlFrame);

      YoGraphicPosition controlPoint = new YoGraphicPosition(bodyName + "ControlPoint", yoControlPoint, 0.01, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic(listName, controlPoint);
      graphics.add(controlPoint);

      if (enablePositionTracking)
      {
         YoGraphicPosition desiredPoint = new YoGraphicPosition(bodyName + "DesiredPoint", yoDesiredPoint, 0.005, YoAppearance.Blue());
         graphicsListRegistry.registerYoGraphic(listName, desiredPoint);
         graphics.add(desiredPoint);
      }

      hideGraphics();
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.defaultAngularWeight = angularWeight;
      this.defaultLinearWeight = linearWeight;
      if (enableOrientationTracking)
         hasAngularWeight.set(angularWeight != null);
      if (enablePositionTracking)
         hasLinearWeight.set(linearWeight != null);
   }

   public void setGains(PID3DGainsReadOnly orientationGains, PID3DGainsReadOnly positionGains)
   {
      this.orientationGains = orientationGains;
      this.positionGains = positionGains;
      if (enableOrientationTracking)
         hasOrientationGains.set(orientationGains != null);
      if (enablePositionTracking)
         hasPositionGains.set(positionGains != null);
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();

      if (!trajectoryDone.getValue())
      {
         boolean isTrajectoryGeneratorEmpty;
         if (!enableOrientationTracking)
         {
            isTrajectoryGeneratorEmpty = positionTrajectoryGenerator.isDone() || positionTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;
         }
         else if (!enablePositionTracking)
         {
            isTrajectoryGeneratorEmpty = orientationTrajectoryGenerator.isDone() || orientationTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;
         }
         else
         {
            isTrajectoryGeneratorEmpty = orientationTrajectoryGenerator.isDone() || orientationTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;
            if (!isTrajectoryGeneratorEmpty)
               isTrajectoryGeneratorEmpty = positionTrajectoryGenerator.isDone() || positionTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;
         }
         if (isTrajectoryGeneratorEmpty)
            fillAndReinitializeTrajectories();
      }

      if (enablePositionTracking)
      {
         positionTrajectoryGenerator.compute(timeInTrajectory);
         positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
         spatialFeedbackControlCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity);
         spatialFeedbackControlCommand.changeFrameAndSetLinearFeedForward(feedForwardLinearAcceleration);
      }

      if (enableOrientationTracking)
      {
         orientationTrajectoryGenerator.compute(timeInTrajectory);
         orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
         spatialFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity);
         spatialFeedbackControlCommand.changeFrameAndSetAngularFeedForward(feedForwardAngularAcceleration);
      }

      if (orientationGains != null)
         spatialFeedbackControlCommand.setOrientationGains(orientationGains);
      if (positionGains != null)
         spatialFeedbackControlCommand.setPositionGains(positionGains);

      // This will improve the tracking with respect to moving trajectory frames.
      if (useBaseFrameForControl.getValue())
      {
         spatialFeedbackControlCommand.setControlBaseFrame(trajectoryFrame);
      }
      else
      {
         spatialFeedbackControlCommand.resetControlBaseFrame();
      }

      // Update the QP weight and selection YoVariables:
      if (defaultLinearWeight != null)
         defaultWeightMatrix.setLinearWeights(defaultLinearWeight);
      if (defaultAngularWeight != null)
         defaultWeightMatrix.setAngularWeights(defaultAngularWeight);
      defaultWeightMatrix.setWeightFrame(null);
      WeightMatrix6D weightMatrix = usingWeightFromMessage.getBooleanValue() ? messageWeightMatrix : defaultWeightMatrix;
      spatialFeedbackControlCommand.setWeightMatrixForSolver(weightMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);

      if (enableOrientationTracking)
      {
         currentAngularWeight.set(weightMatrix.getAngularPart().getXAxisWeight(), weightMatrix.getAngularPart().getYAxisWeight(),
                                  weightMatrix.getAngularPart().getZAxisWeight());
      }
      if (enablePositionTracking)
      {
         currentLinearWeight.set(weightMatrix.getLinearPart().getXAxisWeight(), weightMatrix.getLinearPart().getYAxisWeight(),
                                 weightMatrix.getLinearPart().getZAxisWeight());
      }

      numberOfPointsInQueue.set(pointQueue.size());
      if (enableOrientationTracking)
         numberOfPointsInGenerator.set(orientationTrajectoryGenerator.getCurrentNumberOfWaypoints());
      else
         numberOfPointsInGenerator.set(positionTrajectoryGenerator.getCurrentNumberOfWaypoints());
         
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());

      if (hybridModeActive.getBooleanValue())
      {
         jointControlHelper.doAction(timeInTrajectory);
      }

      updateGraphics();
   }

   @Override
   public void updateGraphics()
   {
      controlPoint.setToZero(controlFrame);
      controlPoint.changeFrame(worldFrame);
      yoControlPoint.set(controlPoint);

      controlOrientation.setToZero(controlFrame);
      controlOrientation.changeFrame(worldFrame);
      yoControlOrientation.set(controlOrientation);

      if (enablePositionTracking)
      {
         desiredPoint.setIncludingFrame(desiredPosition);
         desiredPoint.changeFrame(worldFrame);
         yoDesiredPoint.set(desiredPoint);
      }

      super.updateGraphics();
   }

   private void fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         trajectoryDone.set(true);
         return;
      }

      boolean clearTrajectories = (enablePositionTracking && !positionTrajectoryGenerator.isEmpty())
            || (enableOrientationTracking && !orientationTrajectoryGenerator.isEmpty());

      if (clearTrajectories)
      {
         if (enablePositionTracking)
            positionTrajectoryGenerator.clear(trajectoryFrame);
         if (enableOrientationTracking)
            orientationTrajectoryGenerator.clear(trajectoryFrame);
         lastPointAdded.changeFrame(trajectoryFrame);
         if (enablePositionTracking)
            positionTrajectoryGenerator.appendWaypoint(lastPointAdded);
         if (enableOrientationTracking)
            orientationTrajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      if (enablePositionTracking)
         positionTrajectoryGenerator.changeFrame(trajectoryFrame);
      if (enableOrientationTracking)
         orientationTrajectoryGenerator.changeFrame(trajectoryFrame);

      int currentNumberOfWaypoints = enableOrientationTracking ? orientationTrajectoryGenerator.getCurrentNumberOfWaypoints()
            : positionTrajectoryGenerator.getCurrentNumberOfWaypoints();

      int pointsToAdd = maxPointsInGenerator - currentNumberOfWaypoints;

      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         FrameSE3TrajectoryPoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd); // TODO: get from generators
         if (enablePositionTracking)
            positionTrajectoryGenerator.appendWaypoint(pointToAdd);
         if (enableOrientationTracking)
            orientationTrajectoryGenerator.appendWaypoint(pointToAdd);
      }

      if (enablePositionTracking)
         positionTrajectoryGenerator.initialize();
      if (enableOrientationTracking)
         orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit()
   {
      hideGraphics();
      clear();
   }

   public void holdCurrent()
   {
      clear();
      setWeightsToDefaults();
      resetLastCommandId();
      setTrajectoryStartTimeToCurrentTime();
      queueInitialPoint();

      startTracking();
   }

   public void goToPoseFromCurrent(FramePose3D homePose, double trajectoryTime)
   {
      clear();
      setWeightsToDefaults();
      resetLastCommandId();
      trajectoryFrame = baseFrame;
      setTrajectoryStartTimeToCurrentTime();
      queueInitialPoint();

      homePose.changeFrame(trajectoryFrame);
      FrameSE3TrajectoryPoint homePoint = pointQueue.addLast();
      homePoint.setToZero(trajectoryFrame);
      homePoint.setTime(trajectoryTime);
      homePoint.setPosition(homePose.getPosition());
      homePoint.setOrientation(homePose.getOrientation());

      startTracking();
   }

   public void goToPose(FramePose3D desiredPose, FramePose3D initialPose, double trajectoryTime)
   {
      clear();
      setWeightsToDefaults();
      resetLastCommandId();
      trajectoryFrame = baseFrame;
      setTrajectoryStartTimeToCurrentTime();
      queueInitialPoint(initialPose);

      desiredPose.changeFrame(trajectoryFrame);
      FrameSE3TrajectoryPoint homePoint = pointQueue.addLast();
      homePoint.setToZero(trajectoryFrame);
      homePoint.setTime(trajectoryTime);
      homePoint.setPosition(desiredPose.getPosition());
      homePoint.setOrientation(desiredPose.getOrientation());

      startTracking();
   }

   private void startTracking()
   {
      trajectoryDone.set(false);

      if (enableOrientationTracking && hasOrientationGains.getValue())
      {
         selectionMatrix.resetAngularSelection();
            trackingOrientation.set(true);
      }
      else
      {
         selectionMatrix.clearAngularSelection();
         if (enableOrientationTracking)
            trackingOrientation.set(false);
      }

      if (enablePositionTracking && hasPositionGains.getValue())
      {
         selectionMatrix.resetLinearSelection();
         trackingPosition.set(true);
      }
      else
      {
         selectionMatrix.clearLinearSelection();
         if (enablePositionTracking)
            trackingPosition.set(hasPositionGains.getValue());
      }
   }

   private void setControlFrame(ReferenceFrame controlFrame)
   {
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(bodyFrame);
      this.controlFrame.setPoseAndUpdate(controlFramePose);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
   }

   public void setControlFramePose(RigidBodyTransform controlFrameTransform)
   {
      controlFramePose.setIncludingFrame(bodyFrame, controlFrameTransform);
      this.controlFrame.setPoseAndUpdate(controlFramePose);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
   }

   public void setDefaultControlFrame()
   {
      setControlFrame(defaultControlFrame);
   }

   public ReferenceFrame getControlFrame()
   {
      return controlFrame;
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand command, FramePose3D initialPose)
   {
      if (!enableOrientationTracking)
      {
         PrintTools.error(warningPrefix + "Orientation tracking for this controller has been disabled.");
         return false;
      }

      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (enablePositionTracking && trackingPosition.getValue() && trackingOrientation.getValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking pose. Can not queue orientation trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         clear();
         trajectoryFrame = command.getTrajectoryFrame();
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
         {
            queueInitialPoint(initialPose);
         }

         selectionMatrix.clearLinearSelection();
         selectionMatrix.setAngularPart(command.getSelectionMatrix());

         boolean messageHasValidWeights = true;
         WeightMatrix3D weightMatrix = command.getWeightMatrix();
         messageAngularWeight.set(weightMatrix.getXAxisWeight(), weightMatrix.getYAxisWeight(), weightMatrix.getZAxisWeight());
         if (enablePositionTracking)
            messageLinearWeight.setToZero();
         if (weightMatrix.getWeightFrame() != null)
         {
            messageAngularWeightFrame.set(weightMatrix.getWeightFrame().hashCode());
         }
         else
         {
            messageAngularWeightFrame.set(CommonReferenceFrameIds.NONE.getHashId());
         }
         if (enablePositionTracking)
            messageLinearWeightFrame.set(CommonReferenceFrameIds.NONE.getHashId());
         for (int i = 0; i < 3; i++)
         {
            double weight = messageAngularWeight.getElement(i);
            if (Double.isNaN(weight) || weight < 0.0)
            {
               messageHasValidWeights = false;
            }
         }
         usingWeightFromMessage.set(messageHasValidWeights);
         messageWeightMatrix.setLinearWeights(0.0, 0.0, 0.0);
         messageWeightMatrix.setAngularPart(weightMatrix);

         if (!setAndCheckTrackingFromSelectionMatrices())
         {
            return false;
         }
      }
      else if (command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in " + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame()
               + " without override");
         return false;
      }
      else if (!selectionMatrix.getAngularPart().equals(command.getSelectionMatrix()))
      {
         PrintTools.warn(warningPrefix + "Received a change of selection matrix without an override");
         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryFrame);
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      return true;
   }

   public boolean handleHybridPoseTrajectoryCommand(SE3TrajectoryControllerCommand command, FramePose3D initialPose,
                                                    JointspaceTrajectoryCommand jointspaceCommand, double[] initialJointPositions)
   {
      if (!enableOrientationTracking)
      {
         PrintTools.error(warningPrefix + "Orientation tracking for this controller has been disabled.");
         return false;
      }

      if (!enablePositionTracking)
      {
         PrintTools.error(warningPrefix + "Position tracking for this controller has been disabled.");
         return false;
      }

      if (jointControlHelper == null)
      {
         PrintTools.warn(warningPrefix + "Can not use hybrid mode. Was not created with a jointspace helper.");
         return false;
      }

      if (!handleCommandInternal(jointspaceCommand))
      {
         return false;
      }

      if (!jointControlHelper.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         return false;
      }

      if (!handlePoseTrajectoryCommand(command, initialPose))
      {
         return false;
      }

      hybridModeActive.set(true);
      return true;
   }

   public boolean handlePoseTrajectoryCommand(SE3TrajectoryControllerCommand command, FramePose3D initialPose)
   {
      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      boolean isTrackingPosition = enablePositionTracking && trackingPosition.getValue();
      boolean isTrackingOrientation = enableOrientationTracking && trackingOrientation.getValue();

      if (!override && (!isTrackingPosition && isTrackingOrientation))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         clear();
         trajectoryFrame = command.getTrajectoryFrame();
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
         {
            queueInitialPoint(initialPose);
         }

         selectionMatrix.set(command.getSelectionMatrix());

         boolean messageHasValidWeights = true;
         WeightMatrix6D weightMatrix = command.getWeightMatrix();
         WeightMatrix3D angularWeightMatrix = weightMatrix.getAngularPart();
         WeightMatrix3D linearWeightMatrix = weightMatrix.getLinearPart();
         if (enableOrientationTracking)
         {
            messageAngularWeight.set(angularWeightMatrix.getXAxisWeight(), angularWeightMatrix.getYAxisWeight(), angularWeightMatrix.getZAxisWeight());
            if (angularWeightMatrix.getWeightFrame() != null)
               messageAngularWeightFrame.set(angularWeightMatrix.getWeightFrame().hashCode());
            else
               messageAngularWeightFrame.set(CommonReferenceFrameIds.NONE.getHashId());
         }

         if (enablePositionTracking)
         {
            messageLinearWeight.set(linearWeightMatrix.getXAxisWeight(), linearWeightMatrix.getYAxisWeight(), linearWeightMatrix.getZAxisWeight());
            if (linearWeightMatrix.getWeightFrame() != null)
               messageLinearWeightFrame.set(linearWeightMatrix.getWeightFrame().hashCode());
            else
               messageLinearWeightFrame.set(CommonReferenceFrameIds.NONE.getHashId());
         }

         for (int i = 0; i < 3; i++)
         {
            if (enableOrientationTracking && (Double.isNaN(messageAngularWeight.getElement(i)) || messageAngularWeight.getElement(i) < 0.0))
            {
               messageHasValidWeights = false;
               break;
            }

            if (enablePositionTracking && (Double.isNaN(messageLinearWeight.getElement(i)) || messageLinearWeight.getElement(i) < 0.0))
            {
               messageHasValidWeights = false;
               break;
            }
         }

         usingWeightFromMessage.set(messageHasValidWeights);
         messageWeightMatrix.setAngularPart(angularWeightMatrix);
         messageWeightMatrix.setLinearPart(linearWeightMatrix);

         if (!setAndCheckTrackingFromSelectionMatrices())
         {
            return false;
         }
      }
      else if (command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in ." + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame()
               + " without override");
         return false;
      }
      else if (!selectionMatrix.equals(command.getSelectionMatrix()))
      {
         PrintTools.warn(warningPrefix + "Received a change of selection matrix without an override");
         System.out.println("rbm:\n" + selectionMatrix);
         System.out.println("command:\n" + command.getSelectionMatrix());

         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryFrame);
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      return true;
   }

   public boolean handleEuclideanTrajectoryCommand(EuclideanTrajectoryControllerCommand command, FramePose3D initialPose)
   {
      if (!enablePositionTracking)
      {
         PrintTools.error(warningPrefix + "Position tracking for this controller has been disabled.");
         return false;
      }

      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (!(enablePositionTracking && trackingPosition.getBooleanValue()) && (enableOrientationTracking && trackingOrientation.getBooleanValue())))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         clear();
         trajectoryFrame = command.getTrajectoryFrame();
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
         {
            queueInitialPoint(initialPose);
         }

         selectionMatrix.clearAngularSelection();
         selectionMatrix.setLinearPart(command.getSelectionMatrix());

         boolean messageHasValidWeights = true;
         WeightMatrix3D weightMatrix = command.getWeightMatrix();
         if (enableOrientationTracking)
         {
            messageAngularWeight.setToZero();
            messageAngularWeightFrame.set(CommonReferenceFrameIds.NONE.getHashId());
         }
         messageLinearWeight.set(weightMatrix.getXAxisWeight(), weightMatrix.getYAxisWeight(), weightMatrix.getZAxisWeight());
         if (weightMatrix.getWeightFrame() != null)
         {
            messageLinearWeightFrame.set(weightMatrix.getWeightFrame().hashCode());
         }
         else
         {
            messageLinearWeightFrame.set(CommonReferenceFrameIds.NONE.getHashId());
         }
         for (int i = 0; i < 3; i++)
         {
            double weight = messageLinearWeight.getElement(i);
            if (Double.isNaN(weight) || weight < 0.0)
            {
               messageHasValidWeights = false;
            }
         }
         usingWeightFromMessage.set(messageHasValidWeights);
         messageWeightMatrix.setAngularWeights(0.0, 0.0, 0.0);
         messageWeightMatrix.setLinearPart(weightMatrix);

         if (!setAndCheckTrackingFromSelectionMatrices())
         {
            return false;
         }
      }
      else if (command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in ." + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame()
               + " without override");
         return false;
      }
      else if (!selectionMatrix.getLinearPart().equals(command.getSelectionMatrix()))
      {
         PrintTools.warn(warningPrefix + "Received a change of selection matrix without an override");
         System.out.println("rbm:\n" + selectionMatrix);
         System.out.println("command:\n" + command.getSelectionMatrix());

         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryFrame);
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      return true;
   }

   private boolean setAndCheckTrackingFromSelectionMatrices()
   {
      if (enableOrientationTracking)
      {
         trackingOrientation.set(selectionMatrix.isAngularPartActive());
         
         if (trackingOrientation.getBooleanValue() && !checkOrientationGainsAndWeights())
         {
            return false;
         }
      }

      if (enablePositionTracking)
      {
         trackingPosition.set(selectionMatrix.isLinearPartActive());

         if (trackingPosition.getBooleanValue() && !checkPositionGainsAndWeights())
         {
            return false;
         }
      }

      return true;
   }

   public void getDesiredPose(FramePose3D desiredPoseToPack)
   {
      if (enableOrientationTracking)
         orientationTrajectoryGenerator.getOrientation(desiredOrientation);
      else
         desiredOrientation.setToNaN(positionTrajectoryGenerator.getCurrentTrajectoryFrame());

      if (enablePositionTracking)
         positionTrajectoryGenerator.getPosition(desiredPosition);
      else
         desiredPosition.setToNaN(orientationTrajectoryGenerator.getCurrentTrajectoryFrame());
      desiredPoseToPack.setIncludingFrame(desiredPosition, desiredOrientation);
   }

   public void getDesiredOrientation(FrameQuaternion desiredOrientationToPack)
   {
      if (enableOrientationTracking)
         orientationTrajectoryGenerator.getOrientation(desiredOrientationToPack);
      else
         desiredOrientationToPack.setToNaN(positionTrajectoryGenerator.getCurrentTrajectoryFrame());
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      FeedbackControlCommand<?> feedbackCommandToUse = spatialFeedbackControlCommand;

      if (!enableOrientationTracking)
      {
         spatialFeedbackControlCommand.getIncludingFrame(desiredPosition, desiredLinearVelocity);
         spatialFeedbackControlCommand.getFeedForwardLinearActionIncludingFrame(feedForwardLinearAcceleration);
         SpatialAccelerationCommand spatialAccelerationCommand = spatialFeedbackControlCommand.getSpatialAccelerationCommand();
         spatialAccelerationCommand.getControlFramePoseIncludingFrame(controlPoint, controlOrientation);

         pointFeedbackControlCommand.set(desiredPosition, desiredLinearVelocity);
         pointFeedbackControlCommand.setFeedForwardAction(feedForwardLinearAcceleration);
         pointFeedbackControlCommand.setGains(spatialFeedbackControlCommand.getGains().getPositionGains());
         pointFeedbackControlCommand.setWeightMatrix(spatialAccelerationCommand.getWeightMatrix().getLinearPart());
         pointFeedbackControlCommand.setSelectionMatrix(spatialAccelerationCommand.getSelectionMatrix().getLinearPart());
         pointFeedbackControlCommand.setGainsFrame(spatialFeedbackControlCommand.getLinearGainsFrame());
         pointFeedbackControlCommand.setControlBaseFrame(spatialFeedbackControlCommand.getControlBaseFrame());
         pointFeedbackControlCommand.setBodyFixedPointToControl(controlPoint);
         feedbackCommandToUse = pointFeedbackControlCommand;
      }

      if (!enablePositionTracking)
      {
         spatialFeedbackControlCommand.getIncludingFrame(desiredOrientation, desiredAngularVelocity);
         spatialFeedbackControlCommand.getFeedForwardAngularActionIncludingFrame(feedForwardAngularAcceleration);
         SpatialAccelerationCommand spatialAccelerationCommand = spatialFeedbackControlCommand.getSpatialAccelerationCommand();

         orientationFeedbackControlCommand.set(desiredOrientation, desiredAngularVelocity);
         orientationFeedbackControlCommand.setFeedForwardAction(feedForwardAngularAcceleration);
         orientationFeedbackControlCommand.setGains(spatialFeedbackControlCommand.getGains().getOrientationGains());
         orientationFeedbackControlCommand.setWeightMatrix(spatialAccelerationCommand.getWeightMatrix().getAngularPart());
         orientationFeedbackControlCommand.setSelectionMatrix(spatialAccelerationCommand.getSelectionMatrix().getAngularPart());
         orientationFeedbackControlCommand.setGainsFrame(spatialFeedbackControlCommand.getAngularGainsFrame());
         orientationFeedbackControlCommand.setControlBaseFrame(spatialFeedbackControlCommand.getControlBaseFrame());
         feedbackCommandToUse = orientationFeedbackControlCommand;
      }

      if (hybridModeActive.getBooleanValue())
      {
         feedbackControlCommandList.clear();
         feedbackControlCommandList.addCommand(feedbackCommandToUse);
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
         return feedbackControlCommandList;
      }
      else
      {
         return feedbackCommandToUse;
      }
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());

      if (!enableOrientationTracking)
         feedbackControlCommandList.addCommand(pointFeedbackControlCommand);
      else if (!enablePositionTracking)
         feedbackControlCommandList.addCommand(orientationFeedbackControlCommand);
      else
         feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
      return feedbackControlCommandList;
   }

   /**
    * Returns the spatial feedback control command from this state only. This is used if the control
    * state is not used inside a {@link RigidBodyControlManager} but by itself (this is the case for
    * the feet or the pelvis).
    *
    * @return {@link SpatialFeedbackControlCommand}
    */
   public SpatialFeedbackControlCommand getSpatialFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      if (isEmpty())
      {
         return Double.NEGATIVE_INFINITY;
      }
      else if (pointQueue.isEmpty())
      {
         return orientationTrajectoryGenerator.getLastWaypointTime();
      }
      else
      {
         return pointQueue.peekLast().getTime();
      }
   }

   @Override
   public boolean isEmpty()
   {
      if (!pointQueue.isEmpty())
         return false;
      if (!enableOrientationTracking)
         return positionTrajectoryGenerator.isDone();
      else
         return orientationTrajectoryGenerator.isDone();
   }

   private boolean checkTime(double time)
   {
      boolean timeValid = time > getLastTrajectoryPointTime();
      if (!timeValid)
         PrintTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
      return timeValid;
   }

   private boolean queuePoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      pointQueue.addLast().setIncludingFrame(trajectoryPoint);
      return true;
   }

   private boolean queuePoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      desiredOrientation.setToNaN(trajectoryPoint.getReferenceFrame());
      trajectoryPoint.getOrientation(desiredOrientation);
      desiredAngularVelocity.setToNaN(trajectoryPoint.getReferenceFrame());
      trajectoryPoint.getAngularVelocity(desiredAngularVelocity);

      FrameSE3TrajectoryPoint point = pointQueue.addLast();
      point.setToZero(trajectoryPoint.getReferenceFrame());
      point.setOrientation(desiredOrientation);
      point.setAngularVelocity(desiredAngularVelocity);
      point.setTime(trajectoryPoint.getTime());
      return true;
   }

   private boolean queuePoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      desiredPosition.setToNaN(trajectoryPoint.getReferenceFrame());
      trajectoryPoint.getPosition(desiredPosition);
      desiredLinearVelocity.setToNaN(trajectoryPoint.getReferenceFrame());
      trajectoryPoint.getLinearVelocity(desiredLinearVelocity);

      FrameSE3TrajectoryPoint point = pointQueue.addLast();
      point.setToZero(trajectoryPoint.getReferenceFrame());
      point.setPosition(desiredPosition);
      point.setLinearVelocity(desiredLinearVelocity);
      point.setTime(trajectoryPoint.getTime());
      return true;
   }

   private void queueInitialPoint(FramePose3D initialPose)
   {
      initialPose.changeFrame(trajectoryFrame);
      FrameSE3TrajectoryPoint initialPoint = pointQueue.addLast();
      initialPoint.setToZero(trajectoryFrame);
      initialPoint.setTime(0.0);
      initialPoint.setPosition(initialPose.getPosition());
      initialPoint.setOrientation(initialPose.getOrientation());
   }

   private void queueInitialPoint()
   {
      initialPose.setToZero(controlFrame);
      queueInitialPoint(initialPose);
   }

   private boolean atCapacityLimit()
   {
      if (pointQueue.size() >= maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + maxPoints + " can not execute trajectory.");
         return true;
      }
      return false;
   }

   public void clear()
   {
      selectionMatrix.resetSelection();

      if (enableOrientationTracking)
      {
         orientationTrajectoryGenerator.clear();
         trackingOrientation.set(false);
      }

      if (enablePositionTracking)
      {
         positionTrajectoryGenerator.clear();
         trackingPosition.set(false);
      }

      pointQueue.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      hybridModeActive.set(false);
   }

   public void setWeightsToDefaults()
   {
      usingWeightFromMessage.set(false);
   }

   private boolean checkOrientationGainsAndWeights()
   {
      if (!enableOrientationTracking)
         return true;

      boolean success = true;
      if (!hasAngularWeight.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing angular weight.");
         success = false;
      }
      if (!hasOrientationGains.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing orientation gains.");
         success = false;
      }
      return success;
   }

   private boolean checkPositionGainsAndWeights()
   {
      if (!enablePositionTracking)
         return true;

      boolean success = true;
      if (!hasLinearWeight.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing linear weight.");
         success = false;
      }
      if (!hasPositionGains.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing position gains.");
         success = false;
      }
      return success;
   }

}
