package us.ihmc.avatar.posePlayback;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Set;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJointHolder;

public class PlaybackPose
{
   private final LinkedHashMap<OneDoFJoint, Double> playbackPoseMap;
   private double playBackDelayBeforePose = 1.0;
   private double playBackDuration = 1.0;

   public PlaybackPose(FullRobotModel fullRobotModel, OneDegreeOfFreedomJointHolder oneDegreeOfFreedomJointHolder)
   {
      playbackPoseMap = new LinkedHashMap<OneDoFJoint, Double>();
      OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         String jointName = oneDoFJoint.getName();
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = oneDegreeOfFreedomJointHolder.getOneDegreeOfFreedomJoint(jointName);

         double jointAngle = oneDegreeOfFreedomJoint.getQYoVariable().getDoubleValue();

         playbackPoseMap.put(oneDoFJoint, jointAngle);
      }
   }

   public PlaybackPose(FullRobotModel fullRobotModel, OneDegreeOfFreedomJointHolder oneDegreeOfFreedomJointHolder, double playBackDelayBeforePose,
                       double playbackDuration)
   {
      this(fullRobotModel, oneDegreeOfFreedomJointHolder);
      this.playBackDelayBeforePose = playBackDelayBeforePose;
      this.playBackDuration = playbackDuration;
   }

   public PlaybackPose(LinkedHashMap<OneDoFJoint, Double> playbackPoseMap)
   {
      if (playbackPoseMap == null) throw new RuntimeException("playbackPoseMap == null");
      this.playbackPoseMap = playbackPoseMap;
   }


   public PlaybackPose(OneDoFJoint[] oneDoFJoints, double[] jointAngles, double delay, double duration)
   {
      int length = oneDoFJoints.length;

      if (jointAngles.length != length)
         throw new RuntimeException("jointAngles.length != length");

      playbackPoseMap = new LinkedHashMap<OneDoFJoint, Double>();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints[i];
         playbackPoseMap.put(oneDoFJoint, jointAngles[i]);
      }

      this.playBackDelayBeforePose = delay;
      this.playBackDuration = duration;
   }

   public PlaybackPose(OneDoFJoint[] joints, double playBackDelayBeforePose, double playbackDuration)
   {
      playbackPoseMap = new LinkedHashMap<OneDoFJoint, Double>();

      for (OneDoFJoint oneDoFJoint : joints)
      {
         playbackPoseMap.put(oneDoFJoint, oneDoFJoint.getQ());
      }

      this.playBackDelayBeforePose = playBackDelayBeforePose;
      this.playBackDuration = playbackDuration;
   }

   public PlaybackPose(LinkedHashMap<OneDoFJoint, Double> playbackPoseMap, double playBackDelayBeforePose, double playbackDuration)
   {
      if (playbackPoseMap == null) throw new RuntimeException("playbackPoseMap == null");

      this.playbackPoseMap = playbackPoseMap;
      this.playBackDelayBeforePose = playBackDelayBeforePose;
      this.playBackDuration = playbackDuration;
   }

   public void setRobotAtPose(OneDegreeOfFreedomJointHolder oneDegreeOfFreedomJointHolder)
   {
      Set<OneDoFJoint> oneDoFJoints = playbackPoseMap.keySet();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         String jointName = oneDoFJoint.getName();
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = oneDegreeOfFreedomJointHolder.getOneDegreeOfFreedomJoint(jointName);

         Double value = playbackPoseMap.get(oneDoFJoint);
         oneDegreeOfFreedomJoint.getQYoVariable().set(value);
      }
   }

   public static PlaybackPose morph(PlaybackPose pose1, PlaybackPose pose2, double morphPercentage)
   {
      if (morphPercentage < 0.0)
         morphPercentage = 0.0;
      if (morphPercentage > 1.0)
         morphPercentage = 1.0;


      Set<OneDoFJoint> oneDoFJoints = pose1.playbackPoseMap.keySet();
      LinkedHashMap<OneDoFJoint, Double> playbackPoseMap = new LinkedHashMap<OneDoFJoint, Double>();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         double angleOne = pose1.getJointAngle(oneDoFJoint);
         double angleTwo = pose2.getJointAngle(oneDoFJoint);

         double morphedAngle = (1.0 - morphPercentage) * angleOne + morphPercentage * angleTwo;

         playbackPoseMap.put(oneDoFJoint, morphedAngle);
      }

      return new PlaybackPose(playbackPoseMap);
   }

   public Double getJointAngle(OneDoFJoint oneDoFJoint)
   {
      if (!playbackPoseMap.containsKey(oneDoFJoint)) return null;
      return playbackPoseMap.get(oneDoFJoint);
   }

   public double getPlayBackDelayBeforePose()
   {
      return playBackDelayBeforePose;
   }

   public double getPlayBackDuration()
   {
      return playBackDuration;
   }

   public void setPlaybackDelayBeforePose(double playbackTransitionDelayBeforePose)
   {
      this.playBackDelayBeforePose = playbackTransitionDelayBeforePose;
   }

   public void setPlayBackDuration(double playBackDuration)
   {
      this.playBackDuration = playBackDuration;
   }

   public String toString()
   {
      String ret = "{";

      Set<OneDoFJoint> oneDoFJoints = playbackPoseMap.keySet();

      boolean firstOne = true;
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (!firstOne)
         {
            ret = ret + ", ";
         }

         ret = ret + getJointAngle(oneDoFJoint);
         firstOne = false;
      }

      ret = ret + "}";

      return ret;
   }

   public boolean epsilonEquals(PlaybackPose pose, double jointEpsilon, double timeEpsilon)
   {
      if (this.playbackPoseMap.size() != pose.playbackPoseMap.size())
         return false;

      if (Math.abs(this.playBackDelayBeforePose - pose.getPlayBackDelayBeforePose()) > timeEpsilon)
         return false;

      if (Math.abs(this.playBackDuration - pose.getPlayBackDuration()) > timeEpsilon)
         return false;

      for (OneDoFJoint oneDoFJoint : playbackPoseMap.keySet())
      {
         Double value = pose.playbackPoseMap.get(oneDoFJoint);
         if (value == null)
            return false;

         Double valueTwo = playbackPoseMap.get(oneDoFJoint);
         if (Math.abs(valueTwo - value) > jointEpsilon)
         {
            return false;
         }
      }

      return true;
   }

// public void switchSideDependentValues(FullRobotModel fullRobotModel)
// {
//    double[] newJointAngles = new double[ROSAtlasJointMap.numberOfJoints];
//
//    for (int index = 0; index < ROSAtlasJointMap.numberOfJoints; index++)
//    {
//       int oppositeSideIndex = ROSAtlasJointMapCorrelation.oppositeSideIndex[index];
//       double relativeSign = ROSAtlasJointMapCorrelation.symmetricSignChange[index] ? -1.0 : 1.0;
//       newJointAngles[oppositeSideIndex] = relativeSign * jointAngles[index];
//    }
//
//    for (int i = 0; i < jointAngles.length; i++)
//    {
//       jointAngles[i] = newJointAngles[i];
//       //         System.out.println(i + " \t" + jointAngles[i]);
//    }
//
//    //finger joints
//    double temp = 0;
//    for (int i = 0; i < ROSSandiaJointMap.numberOfJointsPerHand; i++)
//    {
//       temp = fingerJointAngles.get(RobotSide.LEFT)[i];
//       fingerJointAngles.get(RobotSide.LEFT)[i] = fingerJointAngles.get(RobotSide.RIGHT)[i];
//       fingerJointAngles.get(RobotSide.RIGHT)[i] = temp;
//    }
// }

   public PlaybackPose copy()
   {
      LinkedHashMap<OneDoFJoint, Double> playbackPoseMapCopy = new LinkedHashMap<OneDoFJoint, Double>();
      Set<OneDoFJoint> oneDoFJoints = this.playbackPoseMap.keySet();

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         playbackPoseMapCopy.put(oneDoFJoint, playbackPoseMap.get(oneDoFJoint));
      }

      return new PlaybackPose(playbackPoseMapCopy, playBackDelayBeforePose, playBackDuration);
   }

   public double[] getJointAngles()
   {
      Set<OneDoFJoint> oneDoFJoints = this.playbackPoseMap.keySet();

      int size = oneDoFJoints.size();
      int index = 0;
      double[] ret = new double[size];

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         ret[index] = playbackPoseMap.get(oneDoFJoint);
         index++;
      }

      return ret;
   }

   public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
   {
      oneDoFJointsToPack.addAll(playbackPoseMap.keySet());
   }

   public void getJointAngles(ArrayList<OneDoFJoint> oneDoFJoints, double[] jointAnglesToPack)
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         jointAnglesToPack[i] = playbackPoseMap.get(oneDoFJoints.get(i));
      }
   }


}
