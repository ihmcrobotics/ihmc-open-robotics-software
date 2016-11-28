package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point3d;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementOLD.PickUpBallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandDesiredConfigurationTask;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class PutBallInBucketBehavior extends AbstractBehavior
{

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<AbstractBehavior>();
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public PutBallInBucketBehavior(DoubleYoVariable yoTime, PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement,
         HumanoidReferenceFrames referenceFrames, CommunicationBridge outgoingCommunicationBridge, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      setupPipeLine();
   }

   private void setupPipeLine()
   {

      HandDesiredConfigurationTask closeHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.CLOSE,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior);
      HandDesiredConfigurationTask openHand = new HandDesiredConfigurationTask(RobotSide.LEFT, HandConfiguration.OPEN,
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior);

      double[] leftHandBucketLocation1 = new double[] {-0.5609186812662719, -0.39273790125704305, 1.89931104400202, 1.8345084796174007, -1.9173410679363112,
            -0.7657081703756509, -0.7098631227127279};

      ArmTrajectoryMessage leftHandBucketLocation1Message = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandBucketLocation1);

      ArmTrajectoryTask leftHandBucketLocation1Task = new ArmTrajectoryTask(leftHandBucketLocation1Message, atlasPrimitiveActions.leftArmTrajectoryBehavior);

      double[] rightHandBucketLocation1 = new double[] {0.5489321822438367, 0.2899665391571677, 2.096340823983413, -1.2225333451166707, 0.1256161514011733,
            -1.3433026185064938, -1.1994258903111514};

      ArmTrajectoryMessage rightHandBucketLocation1Message = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandBucketLocation1);

      ArmTrajectoryTask rightHandBucketLocation1Task = new ArmTrajectoryTask(rightHandBucketLocation1Message, atlasPrimitiveActions.rightArmTrajectoryBehavior);

      double[] leftHandBucketLocation2 = new double[] {-0.6312858675745908, -0.6560594198655715, 2.026449179186367, 2.0325182474649997, -1.4129369066719957,
            -0.33189990885720594, -1.1435699210219243};

      ArmTrajectoryMessage leftHandBucketLocation2Message = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftHandBucketLocation2);

      ArmTrajectoryTask leftHandBucketLocation2Task = new ArmTrajectoryTask(leftHandBucketLocation2Message, atlasPrimitiveActions.leftArmTrajectoryBehavior);

      double[] rightHandBucketLocation2 = new double[] {0.4765048070153984, 0.305694742754363, 2.173812006625049, -1.4970540590789951, 0.10321456673940527,
            -1.2120648871681976, -1.1591439074587626};

      ArmTrajectoryMessage rightHandBucketLocation2Message = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandBucketLocation2);

      ArmTrajectoryTask rightHandBucketLocation2Task = new ArmTrajectoryTask(rightHandBucketLocation2Message, atlasPrimitiveActions.rightArmTrajectoryBehavior);

      pipeLine.submitSingleTaskStage(rightHandBucketLocation1Task);
      pipeLine.submitSingleTaskStage(leftHandBucketLocation1Task);

      pipeLine.submitSingleTaskStage(rightHandBucketLocation2Task);
      pipeLine.submitSingleTaskStage(leftHandBucketLocation2Task);
 
      pipeLine.submitSingleTaskStage(openHand);
      pipeLine.submitSingleTaskStage(closeHand);
      pipeLine.submitSingleTaskStage(openHand);

      pipeLine.submitSingleTaskStage(leftHandBucketLocation1Task);

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