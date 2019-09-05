package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ArmTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class FireFighterStanceBehavior extends AbstractBehavior
{
   private final FootstepListBehavior footListBehavior;
   private final PelvisHeightTrajectoryBehavior movePelvisBehavior;
   private final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;
   private final ChestTrajectoryBehavior yawChestBehavior;
   private final ArmTrajectoryBehavior leftArmBehavior;
   private final ArmTrajectoryBehavior rightArmBehavior;
   private HumanoidReferenceFrames referenceFrames;

   private final PipeLine<AbstractBehavior> pipeLine;

   public enum BasicStates
   {
      SET_STANCE, MOVE_PELVIS, MOVE_PELVIS2, YAW_CHEST, LEFT_ARM_PRE, LEFT_ARM_FINAL, RIGHT_ARM_FINAL, DONE
   }

   private BasicStates currentState = BasicStates.SET_STANCE;
   private FullHumanoidRobotModel fullRobotModel;

   public FireFighterStanceBehavior(String robotName, String name, YoDouble yoTime, Ros2Node ros2Node,
                                    FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                    WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      footListBehavior = atlasPrimitiveActions.footstepListBehavior;
      movePelvisBehavior = atlasPrimitiveActions.pelvisHeightTrajectoryBehavior;
      yawChestBehavior = atlasPrimitiveActions.chestTrajectoryBehavior;
      leftArmBehavior = atlasPrimitiveActions.leftArmTrajectoryBehavior;
      rightArmBehavior = atlasPrimitiveActions.rightArmTrajectoryBehavior;
      pelvisTrajectoryBehavior = atlasPrimitiveActions.pelvisTrajectoryBehavior;
   }

   private void setUpPipeline()
   {
      pipeLine.clearAll();
      BehaviorAction setStanceTask = new BehaviorAction(footListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

            referenceFrames.updateFrames();
            {
               FramePose3D desiredFootPose = new FramePose3D(referenceFrames.getSoleFrame(RobotSide.LEFT), new Pose3D(.2, .1, 0, 0, 0, 0));
               desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());
               Footstep desiredFootStep = new Footstep(RobotSide.LEFT, desiredFootPose);
               desiredFootsteps.add(desiredFootStep);
            }
            {
               FramePose3D desiredFootPose = new FramePose3D(referenceFrames.getSoleFrame(RobotSide.RIGHT), new Pose3D(-.2, -.1, 0, Math.toRadians(-45), 0, 0));
               desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());

               Footstep desiredFootStep = new Footstep(RobotSide.RIGHT, desiredFootPose);
               desiredFootsteps.add(desiredFootStep);
            }

            PrintTools.debug(this, "Initializing Behavior");
            footListBehavior.initialize();
            footListBehavior.set(desiredFootsteps);
            publishTextToSpeech("Setting Up Stance");
            currentState = BasicStates.SET_STANCE;
         }
      };
      BehaviorAction setStanceTask2 = new BehaviorAction(footListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            ArrayList<Footstep> desiredFootsteps = new ArrayList<Footstep>();

            referenceFrames.updateFrames();
            {
               FramePose3D desiredFootPose = new FramePose3D(referenceFrames.getSoleFrame(RobotSide.LEFT), new Pose3D(.2, .1, 0, 0, 0, 0));
               desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());
               Footstep desiredFootStep = new Footstep(RobotSide.LEFT, desiredFootPose);
               desiredFootsteps.add(desiredFootStep);
            }
            {
               FramePose3D desiredFootPose = new FramePose3D(referenceFrames.getSoleFrame(RobotSide.LEFT), new Pose3D(-.2, -.3, 0, Math.toRadians(-45), 0, 0));
               desiredFootPose.changeFrame(ReferenceFrame.getWorldFrame());

               Footstep desiredFootStep = new Footstep(RobotSide.RIGHT, desiredFootPose);
               desiredFootsteps.add(desiredFootStep);
            }

            PrintTools.debug(this, "Initializing Behavior");
            footListBehavior.initialize();
            footListBehavior.set(desiredFootsteps);
            publishTextToSpeech("Setting Up Stance");
            currentState = BasicStates.SET_STANCE;
         }
      };

      BehaviorAction movePelvisTask = new BehaviorAction(movePelvisBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1, 0.75);
            movePelvisBehavior.setInput(message);
            publishTextToSpeech("Decrease heigth");
            currentState = BasicStates.MOVE_PELVIS;
         }
      };

      BehaviorAction movePelvisTask2 = new BehaviorAction(pelvisTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FrameQuaternion orientation = new FrameQuaternion(referenceFrames.getPelvisFrame(), new Quaternion());
            FramePoint3D p = new FramePoint3D(referenceFrames.getPelvisZUpFrame(), new double[] {0.075, 0.04, 0});
            orientation.changeFrame(ReferenceFrame.getWorldFrame());
            p.changeFrame(ReferenceFrame.getWorldFrame());
            PelvisTrajectoryMessage message = HumanoidMessageTools.createPelvisTrajectoryMessage(2, p, orientation);
            pelvisTrajectoryBehavior.setInput(message);
            publishTextToSpeech("Moving Pelvis To Final Location");
            currentState = BasicStates.MOVE_PELVIS2;
         }
      };

      BehaviorAction moveChestTask = new BehaviorAction(yawChestBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            Quaternion rot = new Quaternion();
            rot.setEuler(0, 0, Math.toRadians(-10));
            ChestTrajectoryMessage chestOrientationPacket = HumanoidMessageTools.createChestTrajectoryMessage(2, rot, referenceFrames.getPelvisZUpFrame());
            yawChestBehavior.setInput(chestOrientationPacket);
            publishTextToSpeech("Setting Chest Yaw");
            currentState = BasicStates.YAW_CHEST;
         }
      };

      BehaviorAction moveLeftPreTask = new BehaviorAction(leftArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[] {-0.575835229010565, -0.8431219813508408, 1.1244418143633323, 1.4875149908528966, 0.9144564803413229,
                  -1.0660904511124556, -2.8798335374002835};
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 2, joints);
            leftArmBehavior.setInput(armTrajectoryMessage);
            publishTextToSpeech("Moving Left Arm To First Location");
            currentState = BasicStates.LEFT_ARM_PRE;
         }
      };
      BehaviorAction moveRightPreTask = new BehaviorAction(rightArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[] {-0.16142948917535702, -0.3569195815223205, 2.792275523125268, -1.026521125798761, -0.4724882991792226,
                  -1.5848189434466957, -0.7292067346614854};
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, joints);
            rightArmBehavior.setInput(armTrajectoryMessage);
            publishTextToSpeech("Moving Right Arm To First Location");
            currentState = BasicStates.RIGHT_ARM_FINAL;
         }
      };
      BehaviorAction moveLeftFinalTask = new BehaviorAction(leftArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[] {0.785398, -1.5708, 3.14159, 1.7671424999999998, 1.6892682660534968, -1.6755335374002835, -2.8798335374002835};
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 3, joints);
            leftArmBehavior.setInput(armTrajectoryMessage);
            publishTextToSpeech("Moving Left Arm To Final Location");
            currentState = BasicStates.LEFT_ARM_FINAL;
         }
      };
      BehaviorAction moveRightFinalTask = new BehaviorAction(rightArmBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            double[] joints = new double[] {0.44195289340641664, 0.023372912207270436, 2.7755155866532912, -1.7857822888113926, 0.38678248792688286,
                  -1.4980698118674458, -0.5046966801690266};
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 3, joints);
            rightArmBehavior.setInput(armTrajectoryMessage);
            publishTextToSpeech("Moving Right Arm To Final Location");
            currentState = BasicStates.RIGHT_ARM_FINAL;
         }
      };

      pipeLine.requestNewStage();
      //pipeLine.submitSingleTaskStage(setStanceTask);
      pipeLine.submitSingleTaskStage(setStanceTask2);

      pipeLine.requestNewStage();
      pipeLine.submitSingleTaskStage(movePelvisTask);
      //      pipeLine.submitTaskForPallelPipesStage(movePelvisBehavior,moveChestTask);
      pipeLine.requestNewStage();

      pipeLine.submitTaskForPallelPipesStage(rightArmBehavior, moveRightPreTask);

      pipeLine.submitTaskForPallelPipesStage(rightArmBehavior, moveLeftPreTask);

      pipeLine.requestNewStage();
      pipeLine.submitTaskForPallelPipesStage(leftArmBehavior, moveLeftFinalTask);
      pipeLine.submitTaskForPallelPipesStage(leftArmBehavior, moveRightFinalTask);
      pipeLine.requestNewStage();

      pipeLine.submitSingleTaskStage(movePelvisTask2);

      //      pipeLine.submitTaskForPallelPipesStage(walkToLocationBehavior, walkToBallTask);

   }

   private FramePose2D getRobotFootPose2d(HumanoidFloatingRootJointRobot robot, RobotSide robotSide)
   {
      List<GroundContactPoint> gcPoints = robot.getFootGroundContactPoints(robotSide);
      Joint ankleJoint = gcPoints.get(0).getParentJoint();
      RigidBodyTransform ankleTransformToWorld = new RigidBodyTransform();
      ankleJoint.getTransformToWorld(ankleTransformToWorld);

      FramePose2D ret = new FramePose2D();
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
      publishTextToSpeech("Getting Ready To Fight Fires");
      setUpPipeline();

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

      currentState = BasicStates.DONE;

      publishTextToSpeech("Ready To Fight Fires");
   }

}
