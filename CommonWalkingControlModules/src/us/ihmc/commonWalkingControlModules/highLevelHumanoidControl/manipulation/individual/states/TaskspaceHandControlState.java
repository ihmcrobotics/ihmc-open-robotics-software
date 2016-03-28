package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * @author twan
 *         Date: 5/9/13
 */
public class TaskspaceHandControlState extends HandControlState
{
   private static final boolean DEBUG = false;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name;
   private final YoVariableRegistry registry;

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

   // viz stuff:
   private final YoFramePose yoDesiredPose;

   // temp stuff:
   private final FramePose desiredPose = new FramePose();
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePose controlFramePose = new FramePose();
   private final PoseReferenceFrame controlFrame;
   private final ReferenceFrame endEffectorFrame;
   private final ReferenceFrame chestFrame;

   private final YoSE3PIDGainsInterface gains;
   private final DoubleYoVariable weight;
   private final RobotSide robotSide;

   private final Map<BaseForControl, ReferenceFrame> baseForControlToReferenceFrameMap;

   public TaskspaceHandControlState(String namePrefix, RobotSide robotSide, RigidBody base, RigidBody endEffector, RigidBody chest,
         YoSE3PIDGainsInterface gains, Map<BaseForControl, ReferenceFrame> baseForControlToReferenceFrameMap, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.TASKSPACE);
      this.robotSide = robotSide;
      this.gains = gains;
      this.baseForControlToReferenceFrameMap = baseForControlToReferenceFrameMap;

      name = namePrefix + FormattingTools.underscoredToCamelCase(getStateEnum().toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      endEffectorFrame = endEffector.getBodyFixedFrame();
      chestFrame = chest.getBodyFixedFrame();

      weight = new DoubleYoVariable(namePrefix + "TaskspaceWeight", registry);
      weight.set(SolverWeightLevels.HAND_TASKSPACE_WEIGHT);

      spatialFeedbackControlCommand.set(base, endEffector);
      spatialFeedbackControlCommand.setPrimaryBase(chest);

      parentRegistry.addChild(registry);

      controlFrame = new PoseReferenceFrame("trackingFrame", endEffectorFrame);
      yoDesiredPose = new YoFramePose(namePrefix + "DesiredPose", worldFrame, registry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix, true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, true, worldFrame, registry);

      for (ReferenceFrame frameToRegister : baseForControlToReferenceFrameMap.values())
      {
         positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
      }

      setupVisualization(namePrefix, yoGraphicsListRegistry);

      privilegedConfigurationCommand.applyPrivilegedConfigurationToSubChain(chest, endEffector);
   }

   private void setupVisualization(String namePrefix, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList list = new YoGraphicsList(name);

      YoGraphicCoordinateSystem desiredPoseViz = new YoGraphicCoordinateSystem(namePrefix + "DesiredPose", yoDesiredPose, 0.3);
      list.add(desiredPoseViz);

      yoGraphicsListRegistry.registerYoGraphicsList(list);
      list.hideYoGraphics();
   }

   public void setWeight(double weight)
   {
      this.weight.set(weight);
   }

   public void holdPositionInChest(ReferenceFrame newControlFrame, boolean initializeToCurrent)
   {
      updateControlFrameAndDesireds(newControlFrame, initializeToCurrent, desiredPosition, desiredOrientation);
      desiredPosition.changeFrame(chestFrame);
      desiredOrientation.changeFrame(chestFrame);
      desiredLinearVelocity.setToZero(chestFrame);
      desiredAngularVelocity.setToZero(chestFrame);

      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.switchTrajectoryFrame(chestFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, desiredPosition, desiredLinearVelocity);
      orientationTrajectoryGenerator.clear();
      orientationTrajectoryGenerator.switchTrajectoryFrame(chestFrame);
      orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
   }

   public boolean handleHandTrajectoryCommand(HandTrajectoryCommand command, ReferenceFrame newControlFrame, boolean initializeToCurrent)
   {

      if (command.getRobotSide() != robotSide)
      {
         PrintTools.warn(this, "Received a " + command.getClass().getSimpleName() + " for the wrong side.");
         return false;
      }

      BaseForControl base = command.getBase();
      ReferenceFrame trajectoryFrame = baseForControlToReferenceFrameMap.get(base);
      if (trajectoryFrame == null)
      {
         PrintTools.error(this, "The base: " + base + " is not handled.");
         return false;
      }
      else if (DEBUG)
      {
         PrintTools.info(this, "Executing hand trajectory in: " + base + ", found corresponding frame: " + trajectoryFrame);
      }

      if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         updateControlFrameAndDesireds(newControlFrame, initializeToCurrent, desiredPosition, desiredOrientation);

         desiredPosition.changeFrame(worldFrame);
         desiredLinearVelocity.setToZero(worldFrame);
         desiredOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         positionTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);

         positionTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.clear();

         positionTrajectoryGenerator.appendWaypoint(0.0, desiredPosition, desiredLinearVelocity);
         orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
      }
      else
      {
         positionTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);

         positionTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.clear();
      }

      positionTrajectoryGenerator.appendWaypoints(command);
      orientationTrajectoryGenerator.appendWaypoints(command);

      positionTrajectoryGenerator.changeFrame(trajectoryFrame);
      orientationTrajectoryGenerator.changeFrame(trajectoryFrame);

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      return true;
   }

   @Override
   public void doAction()
   {
      positionTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.compute(getTimeInCurrentState());

      positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, desiredAcceleration);
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      desiredPose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      yoDesiredPose.setAndMatchFrame(desiredPose);

      spatialFeedbackControlCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, desiredLinearVelocity);
      spatialFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      spatialFeedbackControlCommand.setGains(gains);
      spatialFeedbackControlCommand.setWeightForSolver(weight.getDoubleValue());
   }

   @Override
   public void doTransitionIntoAction()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return positionTrajectoryGenerator.getCurrentTrajectoryFrame();
   }

   private void updateControlFrameAndDesireds(ReferenceFrame newControlFrame, boolean initializeToCurrent, FramePoint desiredPositionToPack,
         FrameOrientation desiredOrientationToPack)
   {
      if (initializeToCurrent)
      {
         desiredPositionToPack.setToZero(newControlFrame);
         desiredOrientationToPack.setToZero(newControlFrame);
      }
      else
      {
         positionTrajectoryGenerator.getPosition(desiredPositionToPack);
         orientationTrajectoryGenerator.getOrientation(desiredOrientationToPack);
         desiredPose.setPoseIncludingFrame(desiredPositionToPack, desiredOrientationToPack);
         changeControlFrame(controlFrame, newControlFrame, desiredPose);
         desiredPose.getPoseIncludingFrame(desiredPositionToPack, desiredOrientationToPack);
      }

      setControlFrameFixedInEndEffector(newControlFrame);
   }

   private final RigidBodyTransform oldTrackingFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newTrackingFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewTrackingFrameToOldTrackingFrame = new RigidBodyTransform();

   private void changeControlFrame(ReferenceFrame oldControlFrame, ReferenceFrame newControlFrame, FramePose framePoseToModify)
   {
      if (oldControlFrame == newControlFrame)
         return;

      framePoseToModify.getPose(oldTrackingFrameDesiredTransform);
      newControlFrame.getTransformToDesiredFrame(transformFromNewTrackingFrameToOldTrackingFrame, oldControlFrame);
      newTrackingFrameDesiredTransform.multiply(oldTrackingFrameDesiredTransform, transformFromNewTrackingFrameToOldTrackingFrame);
      framePoseToModify.setPose(newTrackingFrameDesiredTransform);
   }

   private void setControlFrameFixedInEndEffector(ReferenceFrame controlFrame)
   {
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffectorFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      this.controlFrame.setPoseAndUpdate(controlFramePose);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedConfigurationCommand;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }
}
