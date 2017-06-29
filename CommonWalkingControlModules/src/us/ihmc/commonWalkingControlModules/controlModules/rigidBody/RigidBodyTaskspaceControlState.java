package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class RigidBodyTaskspaceControlState extends RigidBodyControlState
{
   public static final int maxPoints = 1000;
   public static final int maxPointsInGenerator = 5;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private YoOrientationPIDGainsInterface orientationGains = null;
   private YoPositionPIDGainsInterface positionGains = null;

   private final YoFrameVector yoAngularWeight;
   private final YoFrameVector yoLinearWeight;
   private final YoLong yoWeightMatrixAngularFrameID;
   private final YoLong yoWeightMatrixLinearFrameID;
   private final YoFrameVector yoDefaultAngularWeight;
   private final YoFrameVector yoDefaultLinearWeight;
   private final Vector3D angularWeight = new Vector3D();
   private final Vector3D linearWeight = new Vector3D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();

   private final YoBoolean trackingOrientation;
   private final YoBoolean trackingPosition;

   private final YoBoolean hasOrientaionGains;
   private final YoBoolean hasAngularWeight;
   private final YoBoolean hasPositionGains;
   private final YoBoolean hasLinearWeight;

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardLinearAcceleration = new FrameVector(worldFrame);
   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(worldFrame);

   private final RecyclingArrayDeque<FrameSE3TrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(maxPoints, FrameSE3TrajectoryPoint.class);
   private final FrameSE3TrajectoryPoint lastPointAdded = new FrameSE3TrajectoryPoint();

   private final FramePose initialPose = new FramePose();

   private final ReferenceFrame baseFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame defaultControlFrame;
   private final PoseReferenceFrame controlFrame;
   private ReferenceFrame trajectoryFrame;

   private final FramePose controlFramePose = new FramePose();

   private final FramePoint controlPoint = new FramePoint();
   private final YoFramePoint yoControlPoint;
   private final FrameOrientation controlOrientation = new FrameOrientation();
   private final YoFrameOrientation yoControlOrientation;
   private final FramePoint desiredPoint = new FramePoint();
   private final YoFramePoint yoDesiredPoint;

   private final YoBoolean hybridModeActive;
   private final RigidBodyJointControlHelper jointControlHelper;

   public RigidBodyTaskspaceControlState(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator, Collection<ReferenceFrame> trajectoryFrames,
         ReferenceFrame controlFrame, ReferenceFrame baseFrame, YoDouble yoTime, RigidBodyJointControlHelper jointControlHelper,
         YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);
      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "Taskspace";

      this.baseFrame = baseFrame;
      this.trajectoryFrame = baseFrame;
      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.controlFrame = new PoseReferenceFrame(prefix + "ControlFrame", bodyFrame);

      trackingOrientation = new YoBoolean(prefix + "TrackingOrientation", registry);
      trackingPosition = new YoBoolean(prefix + "TrackingPosition", registry);

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);

      spatialFeedbackControlCommand.set(elevator, bodyToControl);
      spatialFeedbackControlCommand.setPrimaryBase(baseBody);
      spatialFeedbackControlCommand.setSelectionMatrixToIdentity();
      defaultControlFrame = controlFrame;
      setControlFrame(defaultControlFrame);

      yoAngularWeight = new YoFrameVector(prefix + "AngularWeight", "_RO", null, registry);
      yoLinearWeight = new YoFrameVector(prefix + "LinearWeight", "_RO", null, registry);
      yoDefaultAngularWeight = new YoFrameVector(prefix + "DefaultAngularWeight", null, registry);
      yoDefaultLinearWeight = new YoFrameVector(prefix + "DefaultLinearWeight", null, registry);
      yoWeightMatrixAngularFrameID = new YoLong(prefix + "WeightMatrixAngularFrameID_RO", null, registry);
      yoWeightMatrixLinearFrameID = new YoLong(prefix + "WeightMatrixLinearFrameID_RO", null, registry);

      yoControlPoint = new YoFramePoint(prefix + "ControlPoint", worldFrame, registry);
      yoControlOrientation = new YoFrameOrientation(prefix + "ControlOrientation", worldFrame, registry);
      yoDesiredPoint = new YoFramePoint(prefix + "DesiredPoint", worldFrame, registry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry);

      if (trajectoryFrames != null)
      {
         for (ReferenceFrame frameToRegister : trajectoryFrames)
         {
            positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
            orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         }
      }

      positionTrajectoryGenerator.registerNewTrajectoryFrame(baseFrame);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(baseFrame);

      hasOrientaionGains = new YoBoolean(prefix + "HasOrientaionGains", registry);
      hasAngularWeight = new YoBoolean(prefix + "HasAngularWeights", registry);
      hasPositionGains = new YoBoolean(prefix + "HasPositionGains", registry);
      hasLinearWeight = new YoBoolean(prefix + "HasLinearWeights", registry);

      pointQueue.clear();

      this.jointControlHelper = jointControlHelper;
      hybridModeActive = new YoBoolean(prefix + "HybridModeActive", registry);

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

      YoGraphicPosition desiredPoint = new YoGraphicPosition(bodyName + "DesiredPoint", yoDesiredPoint, 0.005, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic(listName, desiredPoint);
      graphics.add(desiredPoint);

      hideGraphics();
   }

   public void setWeights(Vector3D angularWeight, Vector3D linearWeight)
   {
      if (angularWeight != null)
      {
         yoDefaultAngularWeight.set(angularWeight);
         hasAngularWeight.set(true);
      }
      else
      {
         yoDefaultAngularWeight.setToZero();
         hasAngularWeight.set(false);
      }

      if (linearWeight != null)
      {
         yoDefaultLinearWeight.set(linearWeight);
         hasLinearWeight.set(true);
      }
      else
      {
         yoDefaultLinearWeight.setToZero();
         hasLinearWeight.set(false);
      }
      yoDefaultAngularWeight.get(this.angularWeight);
      yoDefaultLinearWeight.get(this.linearWeight);

      weightMatrix.setLinearWeights(this.linearWeight);
      weightMatrix.setAngularWeights(this.angularWeight);
   }

   public void setWeight(double weight)
   {
      hasAngularWeight.set(true);
      yoDefaultAngularWeight.set(weight, weight, weight);
      hasLinearWeight.set(true);
      yoDefaultLinearWeight.set(weight, weight, weight);

      yoDefaultAngularWeight.get(this.angularWeight);
      yoDefaultLinearWeight.get(this.linearWeight);

      weightMatrix.setLinearWeights(this.linearWeight);
      weightMatrix.setAngularWeights(this.angularWeight);
   }

   public void setGains(YoOrientationPIDGainsInterface orientationGains, YoPositionPIDGainsInterface positionGains)
   {
      this.orientationGains = orientationGains;
      this.positionGains = positionGains;
      hasOrientaionGains.set(orientationGains != null);
      hasPositionGains.set(positionGains != null);
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = getTimeInTrajectory();

      boolean orientationDone = orientationTrajectoryGenerator.isDone() || orientationTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;
      boolean positionDone = positionTrajectoryGenerator.isDone() || positionTrajectoryGenerator.getLastWaypointTime() <= timeInTrajectory;

      if (!trajectoryDone.getBooleanValue() && (orientationDone || positionDone))
         fillAndReinitializeTrajectories();

      positionTrajectoryGenerator.compute(timeInTrajectory);
      orientationTrajectoryGenerator.compute(timeInTrajectory);

      positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      spatialFeedbackControlCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      spatialFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      if (orientationGains != null)
         spatialFeedbackControlCommand.setGains(orientationGains);
      if (positionGains != null)
         spatialFeedbackControlCommand.setGains(positionGains);

      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
      spatialFeedbackControlCommand.setWeightMatrixForSolver(weightMatrix);

      // GW: commenting this out for now since it breaks some tests:
//      spatialFeedbackControlCommand.setControlBaseFrame(trajectoryFrame);

      //update the qp weight yovariables.
      ReferenceFrame angularSelectionFrame = weightMatrix.getAngularWeightFrame();
      yoWeightMatrixAngularFrameID.set(angularSelectionFrame == null ? controlFrame.getNameBasedHashCode() : angularSelectionFrame.getNameBasedHashCode());
      ReferenceFrame linearSelectionFrame = weightMatrix.getLinearWeightFrame();
      yoWeightMatrixLinearFrameID.set(linearSelectionFrame == null ? controlFrame.getNameBasedHashCode() : linearSelectionFrame.getNameBasedHashCode());

      WeightMatrix3D linearPart = weightMatrix.getLinearPart();
      yoLinearWeight.set(linearPart.getXAxisWeight(), linearPart.getYAxisWeight(), linearPart.getZAxisWeight());
      WeightMatrix3D angularPart = weightMatrix.getAngularPart();
      yoAngularWeight.set(angularPart.getXAxisWeight(), angularPart.getYAxisWeight(), angularPart.getZAxisWeight());

      numberOfPointsInQueue.set(pointQueue.size());
      numberOfPointsInGenerator.set(orientationTrajectoryGenerator.getCurrentNumberOfWaypoints());
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

      desiredPoint.setIncludingFrame(desiredPosition);
      desiredPoint.changeFrame(worldFrame);
      yoDesiredPoint.set(desiredPoint);

      super.updateGraphics();
   }

   private void fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         trajectoryDone.set(true);
         return;
      }

      if (!orientationTrajectoryGenerator.isEmpty())
      {
         positionTrajectoryGenerator.clear(trajectoryFrame);
         orientationTrajectoryGenerator.clear(trajectoryFrame);
         lastPointAdded.changeFrame(trajectoryFrame);
         positionTrajectoryGenerator.appendWaypoint(lastPointAdded);
         orientationTrajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      positionTrajectoryGenerator.changeFrame(trajectoryFrame);
      orientationTrajectoryGenerator.changeFrame(trajectoryFrame);

      int currentNumberOfWaypoints = orientationTrajectoryGenerator.getCurrentNumberOfWaypoints();
      int pointsToAdd = maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         FrameSE3TrajectoryPoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd); // TODO: get from generators
         positionTrajectoryGenerator.appendWaypoint(pointToAdd);
         orientationTrajectoryGenerator.appendWaypoint(pointToAdd);
      }

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      hideGraphics();
   }

   public void holdCurrent()
   {
      clear();
      resetLastCommandId();
      setTrajectoryStartTimeToCurrentTime();
      queueInitialPoint();

      if (hasOrientaionGains.getBooleanValue() && hasPositionGains.getBooleanValue())
      {
         selectionMatrix.resetSelection();
         trajectoryDone.set(false);
         trackingOrientation.set(true);
         trackingPosition.set(true);
      }
      else if (hasOrientaionGains.getBooleanValue())
      {
         selectionMatrix.setToAngularSelectionOnly();
         trajectoryDone.set(false);
         trackingOrientation.set(true);
         trackingPosition.set(false);
      }
   }

   public void goToPoseFromCurrent(FramePose homePose, double trajectoryTime)
   {
      clear();
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

      if (hasOrientaionGains.getBooleanValue() && hasPositionGains.getBooleanValue())
      {
         selectionMatrix.resetSelection();
         trajectoryDone.set(false);
         trackingOrientation.set(true);
         trackingPosition.set(true);
      }
      else if (hasOrientaionGains.getBooleanValue())
      {
         selectionMatrix.setToAngularSelectionOnly();
         trajectoryDone.set(false);
         trackingOrientation.set(true);
         trackingPosition.set(false);
      }
   }

   public void goToPose(FramePose desiredPose, FramePose initialPose, double trajectoryTime)
   {
      clear();
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

      if (hasOrientaionGains.getBooleanValue() && hasPositionGains.getBooleanValue())
      {
         selectionMatrix.resetSelection();
         trajectoryDone.set(false);
         trackingOrientation.set(true);
         trackingPosition.set(true);
      }
      else if (hasOrientaionGains.getBooleanValue())
      {
         selectionMatrix.setToAngularSelectionOnly();
         trajectoryDone.set(false);
         trackingOrientation.set(true);
         trackingPosition.set(false);
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
      controlFramePose.setPoseIncludingFrame(bodyFrame, controlFrameTransform);
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

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> command, FramePose initialPose)
   {
      if (!checkOrientationGainsAndWeights())
         return false;

      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
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

         selectionMatrix.setToAngularSelectionOnly();
         selectionMatrix.setAngularPart(command.getSelectionMatrix());

         WeightMatrix3D weightMatrix = command.getWeightMatrix();
         double xAxisWeight = Double.isNaN(weightMatrix.getXAxisWeight()) ? yoDefaultAngularWeight.getX() : weightMatrix.getXAxisWeight();
         double yAxisWeight = Double.isNaN(weightMatrix.getYAxisWeight()) ? yoDefaultAngularWeight.getY() : weightMatrix.getYAxisWeight();
         double zAxisWeight = Double.isNaN(weightMatrix.getZAxisWeight()) ? yoDefaultAngularWeight.getZ() : weightMatrix.getZAxisWeight();

         weightMatrix.setWeights(xAxisWeight, yAxisWeight, zAxisWeight);
         this.weightMatrix.setAngularPart(weightMatrix);

         trackingOrientation.set(true);
         trackingPosition.set(false);
      }
      else if(command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in " + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame() + " without override");
         return false;
      }
      else if(!selectionMatrix.getAngularPart().equals(command.getSelectionMatrix()))
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

   public boolean handleHybridPoseTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command, FramePose initialPose,
                                              JointspaceTrajectoryCommand<?, ?> jointspaceCommand, double[] initialJointPositions)
   {
      if (jointControlHelper == null)
      {
         PrintTools.warn(warningPrefix + "Can not use hybrid mode. Was not created with a jointspace helper.");
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

   public boolean handlePoseTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command, FramePose initialPose)
   {
      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (!trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         clear();

         selectionMatrix.set(command.getSelectionMatrix());

         WeightMatrix6D weightMatrix = command.getWeightMatrix();
         WeightMatrix3D linearPart = weightMatrix.getLinearPart();
         double linearXAxisWeight = Double.isNaN(linearPart.getXAxisWeight()) ? yoDefaultLinearWeight.getX() : linearPart.getXAxisWeight();
         double linearYAxisWeight = Double.isNaN(linearPart.getYAxisWeight()) ? yoDefaultLinearWeight.getY() : linearPart.getYAxisWeight();
         double linearZAxisWeight = Double.isNaN(linearPart.getZAxisWeight()) ? yoDefaultLinearWeight.getZ() : linearPart.getZAxisWeight();

         weightMatrix.setLinearWeights(linearXAxisWeight, linearYAxisWeight, linearZAxisWeight);

         WeightMatrix3D angularPart = weightMatrix.getAngularPart();
         double angularXAxisWeight = Double.isNaN(angularPart.getXAxisWeight()) ? yoDefaultAngularWeight.getX() : angularPart.getXAxisWeight();
         double angularYAxisWeight = Double.isNaN(angularPart.getYAxisWeight()) ? yoDefaultAngularWeight.getY() : angularPart.getYAxisWeight();
         double angularZAxisWeight = Double.isNaN(angularPart.getZAxisWeight()) ? yoDefaultAngularWeight.getZ() : angularPart.getZAxisWeight();

         weightMatrix.setAngularWeights(angularXAxisWeight, angularYAxisWeight, angularZAxisWeight);

         this.weightMatrix.set(weightMatrix);

         trackingOrientation.set(selectionMatrix.isAngularPartActive());
         trackingPosition.set(selectionMatrix.isLinearPartActive());

         if(trackingOrientation.getBooleanValue() && !checkOrientationGainsAndWeights())
         {
            return false;
         }

         if(trackingPosition.getBooleanValue() && !checkPositionGainsAndWeights())
         {
            return false;
         }

         trajectoryFrame = command.getTrajectoryFrame();
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
         {
            queueInitialPoint(initialPose);
         }
      }
      else if(command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in ." + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame() + " without override");
         return false;
      }
      else if(!selectionMatrix.equals(command.getSelectionMatrix()))
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

   public boolean handleEuclideanTrajectoryCommand(EuclideanTrajectoryControllerCommand<?, ?> command, FramePose initialPose)
   {
      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (!trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         clear();
         selectionMatrix.clearSelection();
         selectionMatrix.setLinearPart(command.getSelectionMatrix());

         WeightMatrix3D linearPart = command.getWeightMatrix();
         double linearXAxisWeight = Double.isNaN(linearPart.getXAxisWeight()) ? yoDefaultLinearWeight.getX() : linearPart.getXAxisWeight();
         double linearYAxisWeight = Double.isNaN(linearPart.getYAxisWeight()) ? yoDefaultLinearWeight.getY() : linearPart.getYAxisWeight();
         double linearZAxisWeight = Double.isNaN(linearPart.getZAxisWeight()) ? yoDefaultLinearWeight.getZ() : linearPart.getZAxisWeight();

         weightMatrix.setLinearWeights(linearXAxisWeight, linearYAxisWeight, linearZAxisWeight);
         this.weightMatrix.set(weightMatrix);

         trackingOrientation.set(selectionMatrix.isAngularPartActive());
         trackingPosition.set(selectionMatrix.isLinearPartActive());

         if(trackingPosition.getBooleanValue() && !checkPositionGainsAndWeights())
         {
            return false;
         }

         trajectoryFrame = command.getTrajectoryFrame();
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
         {
            queueInitialPoint(initialPose);
         }
      }
      else if(command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in ." + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame() + " without override");
         return false;
      }
      else if(!selectionMatrix.equals(command.getSelectionMatrix()))
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

   public void getDesiredPose(FramePose desiredPoseToPack)
   {
      orientationTrajectoryGenerator.getOrientation(desiredOrientation);
      positionTrajectoryGenerator.getPosition(desiredPosition);
      desiredPoseToPack.setPoseIncludingFrame(desiredPosition, desiredOrientation);
   }

   public void getDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(desiredOrientationToPack);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      if (hybridModeActive.getBooleanValue())
      {
         feedbackControlCommandList.clear();
         feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
         feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
         return feedbackControlCommandList;
      }
      else
      {
         return spatialFeedbackControlCommand;
      }
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);
      feedbackControlCommandList.addCommand(jointControlHelper.getJointspaceCommand());
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
      return pointQueue.isEmpty() && orientationTrajectoryGenerator.isDone();
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

   private void queueInitialPoint(FramePose initialPose)
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

   @Override
   public void clear()
   {
      orientationTrajectoryGenerator.clear();
      positionTrajectoryGenerator.clear();
      selectionMatrix.resetSelection();

      weightMatrix.clear();
      yoDefaultLinearWeight.get(linearWeight);
      weightMatrix.setLinearWeights(linearWeight);
      yoDefaultAngularWeight.get(angularWeight);
      weightMatrix.setAngularWeights(angularWeight);

      pointQueue.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      trackingOrientation.set(false);
      trackingPosition.set(false);
      hybridModeActive.set(false);
   }

   private boolean checkPoseGainsAndWeights()
   {
      return checkOrientationGainsAndWeights() && checkPositionGainsAndWeights();
   }

   private boolean checkOrientationGainsAndWeights()
   {
      boolean success = true;
      if (!hasAngularWeight.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing angular weight.");
         success = false;
      }
      if (!hasOrientaionGains.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing orientation gains.");
         success = false;
      }
      return success;
   }

   private boolean checkPositionGainsAndWeights()
   {
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
