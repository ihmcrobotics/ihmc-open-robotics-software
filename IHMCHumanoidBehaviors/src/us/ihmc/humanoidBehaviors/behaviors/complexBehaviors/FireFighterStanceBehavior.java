package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisOrientationTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class FireFighterStanceBehavior extends AbstractBehavior
{
   private final FootstepListBehavior footListBehavior;
   private final PelvisHeightTrajectoryBehavior movePelvisBehavior;
   private final ChestTrajectoryBehavior yawChestBehavior;
   private final ArmTrajectoryBehavior leftArmBehavior;
   private final ArmTrajectoryBehavior rightArmBehavior;
   private HumanoidReferenceFrames referenceFrames;

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<AbstractBehavior>();

   public enum BasicStates
   {
      SET_STANCE, MOVE_PELVIS, YAW_CHEST, LEFT_ARM_PRE, LEFT_ARM_FINAL, RIGHT_ARM_FINAL
   }

   private BasicStates currentState = BasicStates.SET_STANCE;
   private FullHumanoidRobotModel fullRobotModel;

   public FireFighterStanceBehavior(String name, YoDouble yoTime, CommunicationBridge outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(outgoingCommunicationBridge);
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      footListBehavior = atlasPrimitiveActions.footstepListBehavior;
      movePelvisBehavior = atlasPrimitiveActions.pelvisHeightTrajectoryBehavior;
      yawChestBehavior = atlasPrimitiveActions.chestTrajectoryBehavior;
      leftArmBehavior = atlasPrimitiveActions.leftArmTrajectoryBehavior;
      rightArmBehavior = atlasPrimitiveActions.rightArmTrajectoryBehavior;

      setUpPipeline();
   }

   private void setUpPipeline()
   {
      BehaviorAction setStanceTask = new BehaviorAction(footListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();


            referenceFrames.updateFrames();
            {
               FramePose desiredFootPose = new FramePose(referenceFrames.getSoleFrame(RobotSide.LEFT), new Pose3D(.2,.1, 0,0,0,0));
               desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());
               Footstep desiredFootStep = new Footstep(fullRobotModel.getFoot(RobotSide.LEFT),RobotSide.LEFT, desiredFootPose);
              desiredFootsteps.add(desiredFootStep);
            }
            {
               FramePose desiredFootPose = new FramePose(referenceFrames.getSoleFrame(RobotSide.RIGHT), new Pose3D(-.2,-.1, 0,Math.toRadians(-45),0,0));
               desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());

               Footstep desiredFootStep = new Footstep(fullRobotModel.getFoot(RobotSide.RIGHT),RobotSide.RIGHT, desiredFootPose);
              desiredFootsteps.add(desiredFootStep);
            }

            PrintTools.debug(this, "Initializing Behavior");
            footListBehavior.initialize();
            footListBehavior.set(desiredFootsteps);

            currentState = BasicStates.SET_STANCE;
         }
      };

      BehaviorAction movePelvisTask = new BehaviorAction(movePelvisBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage(1, 0.75);
            movePelvisBehavior.setInput(message);
            currentState = BasicStates.MOVE_PELVIS;
         }
      };

      BehaviorAction moveChestTask = new BehaviorAction(yawChestBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, 0, Math.toRadians(-10));
            ChestTrajectoryMessage chestOrientationPacket = new ChestTrajectoryMessage(2, rot, referenceFrames.getPelvisZUpFrame());
            yawChestBehavior.setInput(chestOrientationPacket);
            currentState = BasicStates.YAW_CHEST;
         }
      };

      BehaviorAction moveLeftPreTask = new BehaviorAction(leftArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[]{-0.575835229010565, -0.8431219813508408, 1.1244418143633323, 1.4875149908528966, 0.9144564803413229, -1.0660904511124556, -2.8798335374002835};
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, joints);
            leftArmBehavior.setInput(armTrajectoryMessage);
            currentState = BasicStates.LEFT_ARM_PRE;
         }
      };
      BehaviorAction moveRightPreTask = new BehaviorAction(rightArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[]{-0.16142948917535702, -0.3569195815223205, 2.792275523125268, -1.026521125798761, -0.4724882991792226, -1.5848189434466957, -0.7292067346614854};
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, joints);
            rightArmBehavior.setInput(armTrajectoryMessage);
            currentState = BasicStates.RIGHT_ARM_FINAL;
         }
      };
      BehaviorAction moveLeftFinalTask = new BehaviorAction(leftArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[]{0.785398, -1.5708, 3.14159, 1.7671424999999998, 1.6892682660534968, -1.6755335374002835, -2.8798335374002835};
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, joints);
            leftArmBehavior.setInput(armTrajectoryMessage);
            currentState = BasicStates.LEFT_ARM_FINAL;
         }
      };
      BehaviorAction moveRightFinalTask = new BehaviorAction(rightArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[]{0.44195289340641664, 0.023372912207270436, 2.7755155866532912, -1.7857822888113926, 0.38678248792688286, -1.4980698118674458, -0.5046966801690266};
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, joints);
            rightArmBehavior.setInput(armTrajectoryMessage);
            currentState = BasicStates.RIGHT_ARM_FINAL;
         }
      };

      pipeLine.requestNewStage();
      pipeLine.submitSingleTaskStage(setStanceTask);
      pipeLine.requestNewStage();
//      pipeLine.submitTaskForPallelPipesStage(movePelvisBehavior,movePelvisTask);
      pipeLine.submitTaskForPallelPipesStage(movePelvisBehavior,moveChestTask);
      pipeLine.submitTaskForPallelPipesStage(leftArmBehavior,moveLeftPreTask);
      pipeLine.submitTaskForPallelPipesStage(leftArmBehavior,moveLeftPreTask);

      pipeLine.requestNewStage();
      pipeLine.submitTaskForPallelPipesStage(leftArmBehavior,moveLeftFinalTask);
      pipeLine.submitTaskForPallelPipesStage(leftArmBehavior,moveRightFinalTask);


//      pipeLine.submitSingleTaskStage(setStanceTask);
//      pipeLine.submitSingleTaskStage(setStanceTask);

      
      //      pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, walkToBallTask);

   }

   private FramePose2d getRobotFootPose2d(HumanoidFloatingRootJointRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose2d ret = new FramePose2d();
      ret.setIncludingFrame(ReferenceFrame.getWorldFrame(), ankleTransformToWorld, false);

      return ret;
   }

  

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void onBehaviorEntered()
   {
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

   @Override
   public void onBehaviorExited()
   {
   }

}
