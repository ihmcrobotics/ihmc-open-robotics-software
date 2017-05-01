package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class UserPelvisOrientationManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final BooleanYoVariable pelvisIsFollowingUserTrajectory = new BooleanYoVariable("PelvisIsFollowingUserTrajectory", registry);

   private final FramePose tempPose = new FramePose();
   private final RigidBodyTaskspaceControlState taskspaceControlState;

   public UserPelvisOrientationManager(YoOrientationPIDGainsInterface gains, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      RigidBody pelvis = controllerToolbox.getFullRobotModel().getPelvis();
      RigidBody elevator = controllerToolbox.getFullRobotModel().getElevator();
      Collection<ReferenceFrame> trajectoryFrames = controllerToolbox.getTrajectoryFrames();
      ReferenceFrame pelvisFixedFrame = pelvis.getBodyFixedFrame();
      ReferenceFrame baseFrame = controllerToolbox.getReferenceFrames().getMidFootZUpGroundFrame();
      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      
      taskspaceControlState = new RigidBodyTaskspaceControlState(pelvis, elevator, elevator, trajectoryFrames, pelvisFixedFrame, baseFrame, yoTime, graphicsListRegistry, registry);
      taskspaceControlState.setGains(gains, null);
      
      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3D angularWeight)
   {
      taskspaceControlState.setWeights(angularWeight, null);
   }

   public void compute()
   {
      checkIfPelvisInUserMode();
      taskspaceControlState.doAction();
   }

   public void handlePelvisOrientationTrajectoryCommands(PelvisOrientationTrajectoryCommand command)
   {
      computeInitialPose(tempPose);
      taskspaceControlState.handleOrientationTrajectoryCommand(command, tempPose);
      pelvisIsFollowingUserTrajectory.set(true);
   }

   public boolean isFollowingUserTrajectory()
   {
      return pelvisIsFollowingUserTrajectory.getBooleanValue();
   }

   public void disableUserTrajectory()
   {
      pelvisIsFollowingUserTrajectory.set(false);
   }

   public void getCurrentDesiredOrientation(FrameOrientation desiredOrientation)
   {
      if (pelvisIsFollowingUserTrajectory.getBooleanValue())
      {
         desiredOrientation.setToNaN();
      }
      else
      {
         taskspaceControlState.getDesiredPose(tempPose);
         tempPose.getOrientationIncludingFrame(desiredOrientation);
      }
   }

   private void computeInitialPose(FramePose initialPose)
   {
      if (pelvisIsFollowingUserTrajectory.getBooleanValue())
      {
         taskspaceControlState.getDesiredPose(initialPose);
      }
      else
      {
         initialPose.setToZero(taskspaceControlState.getControlFrame());
      }
   }
   
   private void checkIfPelvisInUserMode()
   {
      if (!pelvisIsFollowingUserTrajectory.getBooleanValue())
      {
         throw new RuntimeException(getClass().getSimpleName() + " is not active. Can not call compute before swintching to user mode.");
      }
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return taskspaceControlState.getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return taskspaceControlState.getFeedbackControlCommand();
   }
}
