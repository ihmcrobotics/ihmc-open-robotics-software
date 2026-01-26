package us.ihmc.zulu.parameters.controller;

import us.ihmc.zulu.ZuluJointMap;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.LinkedHashMap;
import java.util.Map;

public class ZuluStandPrepSetPoints implements WholeBodySetpointParameters
{
   private final Map<String, Double> setPoints = new LinkedHashMap<>();

   public ZuluStandPrepSetPoints(ZuluJointMap jointMap)
   {
      setPoints.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setPoints.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      setPoints.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0);
      setPoints.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), robotSide.negateIfRightSide(0.0));
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), robotSide.negateIfRightSide(0.1));
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.45);
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.7);
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.28);
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), robotSide.negateIfLeftSide(0.0));

         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.4));
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(-0.4));
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.7);
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -1.9);
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_YAW), 0.0);
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_YAW), 0.0);
      }
   }

   @Override
   public double getSetpoint(String jointName)
   {
      if (setPoints.containsKey(jointName))
         return setPoints.get(jointName);
      else
         return 0.0;
   }
}
