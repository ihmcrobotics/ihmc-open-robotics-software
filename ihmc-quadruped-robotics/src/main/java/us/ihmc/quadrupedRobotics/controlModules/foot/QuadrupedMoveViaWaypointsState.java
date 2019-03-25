package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3DReadOnly zeroVector3D = new FrameVector3D(worldFrame);

   // Yo variables
   private final YoDouble robotTime;

   private final MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   private final FrameEuclideanTrajectoryPointList trajectoryPointList = new FrameEuclideanTrajectoryPointList();

   private final QuadrupedFootControlModuleParameters parameters;

   private final VirtualForceCommand virtualForceCommand = new VirtualForceCommand();

   private final QuadrupedControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FrameVector3D desiredFootVelocity = new FrameVector3D();
   private final FrameVector3D desiredFootAcceleration = new FrameVector3D();

   private WholeBodyControllerCoreMode controllerCoreMode = WholeBodyControllerCoreMode.VIRTUAL_MODEL;
   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private double taskStartTime;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      ReferenceFrame soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      this.parameters = controllerToolbox.getFootControlModuleParameters();
      robotTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      // Create waypoint trajectory
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       worldFrame, registry);

      RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      this.controllerCoreMode = controllerCoreMode;
      feedbackControlCommand.setControlMode(controllerCoreMode);
   }

   public void handleWaypointList(FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.set(trajectoryPointList);
   }

   public void initialize()
   {
      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.getFootContactState(robotQuadrant).clear();

      initialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
      quadrupedWaypointsPositionTrajectoryGenerator.getPosition(desiredFootPosition);
      quadrupedWaypointsPositionTrajectoryGenerator.getVelocity(desiredFootVelocity);
      quadrupedWaypointsPositionTrajectoryGenerator.getAcceleration(desiredFootAcceleration);

      desiredFootPosition.changeFrame(worldFrame);
      desiredFootVelocity.changeFrame(worldFrame);
      desiredFootAcceleration.changeFrame(worldFrame);

      if (controllerCoreMode == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         feedbackControlCommand.setInverseDynamics(desiredFootPosition, desiredFootVelocity, desiredFootAcceleration);
      else if (controllerCoreMode == WholeBodyControllerCoreMode.VIRTUAL_MODEL)
         feedbackControlCommand.setVirtualModelControl(desiredFootPosition, desiredFootVelocity, zeroVector3D);
      else
         throw new UnsupportedOperationException("Unsupported control mode: " + controllerCoreMode);
      feedbackControlCommand.setGains(parameters.getSolePositionGains());
      feedbackControlCommand.setWeightsForSolver(parameters.getSolePositionWeights());
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.changeFrame(trajectoryPointList.getReferenceFrame());

      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoints(trajectoryPointList);
      if (trajectoryPointList.getNumberOfTrajectoryPoints() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > trajectoryPointList.getTrajectoryTime())
         return virtualForceCommand;
      else
         return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > trajectoryPointList.getTrajectoryTime())
         return null;
      else
         return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return feedbackControlCommand;
   }
}
