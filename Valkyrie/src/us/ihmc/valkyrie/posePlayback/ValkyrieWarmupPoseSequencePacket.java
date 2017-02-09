package us.ihmc.valkyrie.posePlayback;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.avatar.posePlayback.PlaybackPose;
import us.ihmc.avatar.posePlayback.PlaybackPoseSequence;
import us.ihmc.avatar.posePlayback.PlaybackPoseSequenceReader;
import us.ihmc.avatar.posePlayback.PosePlaybackPacket;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.valkyrie.configuration.YamlWithIncludesLoader;

public class ValkyrieWarmupPoseSequencePacket implements PosePlaybackPacket
{
   private final double trajectoryTime = 2.0;
   private final double delayTime = 0.25;
   private final double initialGainScaling;
   
   private final LinkedHashMap<OneDoFJoint, Double> kps = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, Double> kds = new LinkedHashMap<>();
   private final PlaybackPoseSequence playbackPoseSequence;

   public ValkyrieWarmupPoseSequencePacket(FullHumanoidRobotModel fullRobotModel, double initialGainScaling)
   {
      @SuppressWarnings("unchecked")
      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) YamlWithIncludesLoader.load("standPrep", "gains.yaml");

      this.initialGainScaling = initialGainScaling;
      playbackPoseSequence = new PlaybackPoseSequence(fullRobotModel);
      
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         Map<String, Double> jointGains = gainMap.get(joint.getName());
         
         kps.put(joint, jointGains.get("kp"));
         kds.put(joint, jointGains.get("kd"));
      }

      List<Double> listOfHipAdductorPoses = new ArrayList<>();
      listOfHipAdductorPoses.add(0.0);
      listOfHipAdductorPoses.add(-0.4);
      listOfHipAdductorPoses.add(0.0);
      listOfHipAdductorPoses.add(-0.4);
      listOfHipAdductorPoses.add(0.0);

      for (Double angle : listOfHipAdductorPoses)
      {
         LinkedHashMap<OneDoFJoint, Double> map = new LinkedHashMap<>();
         for (RobotSide robotSide : RobotSide.values)
         {
            map.put(fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_ROLL), angle);
         }
         
         PlaybackPose playbackPose = new PlaybackPose(map);
         playbackPose.setPlaybackDelayBeforePose(delayTime);
         playbackPose.setPlayBackDuration(trajectoryTime);
         
         playbackPoseSequence.addPose(playbackPose);
      }
   }
   
   
   public ValkyrieWarmupPoseSequencePacket(String filename, FullRobotModel fullRobotModel, double initialGainScaling)
   {
      @SuppressWarnings("unchecked")
      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) YamlWithIncludesLoader.load("standPrep", "gains.yaml");

      this.initialGainScaling = initialGainScaling;
//      playbackPoseSequence = PlaybackPoseSequenceReader.readFromFile(fullRobotModel, filename);
      playbackPoseSequence = PlaybackPoseSequenceReader.readFromInputStream(fullRobotModel, getClass().getClassLoader().getResourceAsStream(filename));
      for (OneDoFJoint joint : fullRobotModel.getOneDoFJoints())
      {
         Map<String, Double> jointGains = gainMap.get(joint.getName());
         
         System.out.println(joint.getName());
         
         kps.put(joint, jointGains.get("kp"));
         kds.put(joint, jointGains.get("kd"));
      }
   }

   @Override
   public Map<OneDoFJoint, Double> getJointKps()
   {
      return kps;
   }

   @Override
   public Map<OneDoFJoint, Double> getJointKds()
   {
      return kds;
   }

   @Override
   public PlaybackPoseSequence getPlaybackPoseSequence()
   {
      return playbackPoseSequence;
   }

   @Override
   public double getInitialGainScaling()
   {
      return initialGainScaling;
   }

}
