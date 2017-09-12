package us.ihmc.avatar.posePlayback;

import java.util.LinkedHashMap;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class PosePlaybackExampleSequence
{
   private static final double[] pose1 = new double[]
   {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
   };
   private static final double[] pose2 = new double[]
   {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0, 0.0, 0.0,
      -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0
   };
   private static final double[] pose3 = new double[]
   {
      0.0, 0.8113385826771654, 0.0, 0.0, 0.0, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.2165354330708662, 0.0, -0.8547244094488189, 0.0, 0.0,
      0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0
   };
   private static final double[] pose4 = new double[]
   {
      0.0, 0.8113385826771654, 0.0, 0.0, 0.5307086614173226, 0.10748031496062997, -0.8547244094488189, 0.0, 0.0, 0.0, -0.32000000000000006, 0.0,
      -0.8547244094488189, 0.0, 0.0, 0.0, 0.0, -0.7283629133858267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7063789763779529, 0.0, 0.0, 0.0, 0.0
   };

   public static PlaybackPoseSequence createExamplePoseSequenceMoveArm(FullHumanoidRobotModel fullRobotModel, double poseDelay, double trajectoryTime)
   {
      PlaybackPoseSequence sequence = new PlaybackPoseSequence(fullRobotModel);

      double[] elbowAngles = new double[]{0.0, 0.37, 0.1};

      for (int i = 0; i < elbowAngles.length; i++)
      {
         LinkedHashMap<OneDoFJoint, Double> pose = new LinkedHashMap<>();

         OneDoFJoint leftElbowPitch = fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH);
         pose.put(leftElbowPitch, elbowAngles[i]);

         PlaybackPose posePlaybackRobotPose = new PlaybackPose(pose, poseDelay, trajectoryTime);
         sequence.addPose(posePlaybackRobotPose);
      }
      
      return sequence;
   }
   
// public static PosePlaybackRobotPoseSequence createExampleSequenceEmpty()
// {
//    PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
//    return sequence;
// }
// 
// public static PosePlaybackRobotPoseSequence createExampleSequenceOnePose()
// {
//    PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
//    
//    sequence.addPose(new PosePlaybackRobotPose(pose1));
//    
//    return sequence;
// }
// 
// public static PosePlaybackRobotPoseSequence createExampleSequenceTwoPoses()
// {
//    PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
//    
//    sequence.addPose(new PosePlaybackRobotPose(pose1));
//    sequence.addPose(new PosePlaybackRobotPose(pose2));
//    
//    return sequence;
// }
// 
// public static PosePlaybackRobotPoseSequence createExampleSequenceThreePoses()
// {
//    PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence();
//    
//    sequence.addPose(new PosePlaybackRobotPose(pose1));
//    sequence.addPose(new PosePlaybackRobotPose(pose2));
//    sequence.addPose(new PosePlaybackRobotPose(pose3));
//    
//    return sequence;
// }
// 
// public static PosePlaybackRobotPoseSequence createExampleSequenceFourPoses(FullRobotModel fullRobotModel)
// {
//    PosePlaybackRobotPoseSequence sequence = new PosePlaybackRobotPoseSequence(fullRobotModel);
//    
//    sequence.addPose(new PosePlaybackRobotPose(pose1));
//    sequence.addPose(new PosePlaybackRobotPose(pose2));
//    sequence.addPose(new PosePlaybackRobotPose(pose3));
//    sequence.addPose(new PosePlaybackRobotPose(pose4));
//    
//    return sequence;
// }

   public static PlaybackPoseSequence createRandomPlaybackPoseSequence(Random random, FullRobotModel fullRobotModel, int numberOfPoses, double poseDelay, double trajectoryTime)
   {
      PlaybackPoseSequence sequence = new PlaybackPoseSequence(fullRobotModel);

      for (int i = 0; i < numberOfPoses; i++)
      {
         sequence.addPose(createRandomPosePlaybackRobotPose(random, fullRobotModel, poseDelay, trajectoryTime));
      }
      
      return sequence;
   }

   public static PlaybackPose createRandomPosePlaybackRobotPose(Random random, FullRobotModel fullRobotModel, double poseDelay, double trajectoryTime)
   {
      LinkedHashMap<OneDoFJoint, Double> pose = new LinkedHashMap<>();

      OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         
         if (jointLimitLower < -Math.PI) jointLimitLower = -Math.PI;
         if (jointLimitUpper > Math.PI) jointLimitUpper = Math.PI;
         pose.put(joint, RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper));
      }

      return new PlaybackPose(pose, poseDelay, trajectoryTime);
   }
}
