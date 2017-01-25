package us.ihmc.commonWalkingControlModules.controlModules.chest;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.io.printing.PrintTools;

public class TaskspaceChestControlState extends ChestControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final YoFrameVector yoChestAngularWeight = new YoFrameVector("chestWeight", null, registry);
   private final Vector3d chestAngularWeight = new Vector3d();
   private final YoOrientationPIDGainsInterface gains;

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final ReferenceFrame pelvisZUpFrame;

   public TaskspaceChestControlState(HighLevelHumanoidControllerToolbox humanoidControllerToolbox, YoOrientationPIDGainsInterface gains, Vector3d angularWeight, YoVariableRegistry parentRegistry)
   {
      super(ChestControlMode.TASK_SPACE);

      this.gains = gains;

      FullRobotModel fullRobotModel = humanoidControllerToolbox.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      pelvisZUpFrame = humanoidControllerToolbox.getPelvisZUpFrame();

      yoChestAngularWeight.set(angularWeight);
      yoChestAngularWeight.get(chestAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(chestAngularWeight);
      orientationFeedbackControlCommand.set(elevator, chest);
      orientationFeedbackControlCommand.setGains(gains);

      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", true, pelvisZUpFrame, registry);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3d weights)
   {
      yoChestAngularWeight.set(weights);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }

   @Override
   public void doAction()
   {
      orientationTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      desiredAngularVelocity.setToZero(worldFrame);
      feedForwardAngularAcceleration.setToZero(worldFrame);
      orientationFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      orientationFeedbackControlCommand.setGains(gains);
      yoChestAngularWeight.get(chestAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(chestAngularWeight);
   }

   @Override
   public void doTransitionIntoAction()
   {
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {

   }

   public void holdOrientation(FrameOrientation orientationToHold)
   {
      desiredOrientation.setIncludingFrame(orientationToHold);
      desiredOrientation.changeFrame(pelvisZUpFrame);
      desiredAngularVelocity.setToZero(pelvisZUpFrame);

      orientationTrajectoryGenerator.switchTrajectoryFrame(pelvisZUpFrame);
      orientationTrajectoryGenerator.clear();
      orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
   }

   public void handleChestTrajectoryCommand(ChestTrajectoryCommand command, FrameOrientation initialOrientation)
   {
      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         overrideTrajectory(command, initialOrientation);
         break;
      case QUEUE:
         queueTrajectory(command, initialOrientation);
         break;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         return;
      }
   }

   private void queueTrajectory(ChestTrajectoryCommand command, FrameOrientation initialOrientation)
   {
      throw new RuntimeException("Implement me!");
   }

   private void overrideTrajectory(ChestTrajectoryCommand command, FrameOrientation initialOrientation)
   {
      if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         desiredOrientation.setIncludingFrame(initialOrientation);
         desiredOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         orientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
      }
      else
      {
         orientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
         orientationTrajectoryGenerator.clear();
      }

      orientationTrajectoryGenerator.appendWaypoints(command);
      orientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      orientationTrajectoryGenerator.initialize();
   }

   public void goToHome(double trajectoryTime, FrameOrientation initialOrientation)
   {
      desiredOrientation.setIncludingFrame(initialOrientation);
      desiredOrientation.changeFrame(pelvisZUpFrame);
      desiredAngularVelocity.setToZero(pelvisZUpFrame);

      orientationTrajectoryGenerator.switchTrajectoryFrame(pelvisZUpFrame);
      orientationTrajectoryGenerator.clear();
      orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);

      desiredOrientation.setToZero(pelvisZUpFrame);
      orientationTrajectoryGenerator.appendWaypoint(trajectoryTime, desiredOrientation, desiredAngularVelocity);
      orientationTrajectoryGenerator.initialize();
   }

   public void getDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(desiredOrientationToPack);
   }

}
