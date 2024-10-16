package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.List;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestSetHeightBehavior extends AbstractBehavior
{
   private final PelvisHeightTrajectoryBehavior movePelvisBehavior;
   private HumanoidReferenceFrames referenceFrames;

   private final PipeLine<AbstractBehavior> pipeLine;

   public enum BasicStates
   {
      SET_STANCE, MOVE_PELVIS, MOVE_PELVIS2, YAW_CHEST, LEFT_ARM_PRE, LEFT_ARM_FINAL, RIGHT_ARM_FINAL, DONE
   }

   private BasicStates currentState = BasicStates.SET_STANCE;
   private FullHumanoidRobotModel fullRobotModel;

   public TestSetHeightBehavior(String robotName, String name, YoDouble yoTime, ROS2Node ros2Node,
                                    FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                    WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      movePelvisBehavior = atlasPrimitiveActions.pelvisHeightTrajectoryBehavior;
    
   }

   private void setUpPipeline()
   {
      pipeLine.clearAll();
    
      BehaviorAction movePelvisTask = new BehaviorAction(movePelvisBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            
            PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1, 0.8, ReferenceFrame.getWorldFrame(), referenceFrames.getMidFeetZUpFrame());

            
            movePelvisBehavior.setInput(message);
            publishTextToSpeech("Decrease heigth");
            currentState = BasicStates.MOVE_PELVIS;
         }
      };

     

      pipeLine.requestNewStage();
      //pipeLine.submitSingleTaskStage(setStanceTask);
      pipeLine.submitSingleTaskStage(movePelvisTask);

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
      publishTextToSpeech("Getting Ready Test height");
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

   }

}
