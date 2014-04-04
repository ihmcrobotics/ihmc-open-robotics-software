package us.ihmc.valkyrie.posePlayback;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.posePlayback.PosePlaybackPacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrie.roboNet.schedule.YamlWithIncludesLoader;

public class ValkyrieWarmupPoseSequencePacket implements PosePlaybackPacket
{
   private final double trajectoryTime = 1.0;
   private final LinkedHashMap<OneDoFJoint, Double> kps = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, Double> kds = new LinkedHashMap<>();
   private final List<Map<OneDoFJoint, Double>> listOfPosesToPlayback = new ArrayList<>();
   private final List<Double> trajectoryTimes = new ArrayList<>();

   public ValkyrieWarmupPoseSequencePacket(FullRobotModel fullRobotModel)
   {
      @SuppressWarnings("unchecked")
      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) YamlWithIncludesLoader.load(ValkyrieConfigurationRoot.class, "standPrep", "gains.yaml");

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
         listOfPosesToPlayback.add(map);
         trajectoryTimes.add(trajectoryTime);
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
   public List<Double> getTrajectoryTimes()
   {
      return trajectoryTimes;
   }

   @Override
   public List<Map<OneDoFJoint, Double>> getListOfPosesToPlayback()
   {
      return listOfPosesToPlayback;
   }

}
