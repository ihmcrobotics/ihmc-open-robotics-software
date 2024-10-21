package us.ihmc.atlas.parameters;

import java.util.HashMap;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.robotics.partNames.ArmJointName;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.commons.robotics.partNames.NeckJointName;
import us.ihmc.commons.robotics.partNames.SpineJointName;
import us.ihmc.commons.robotics.robotSide.RobotSide;

public class AtlasStandPrepSetpoints implements WholeBodySetpointParameters
{
   private final HashMap<String, Double> setPoints = new HashMap<>();
   private final AtlasJointMap jointMap;

   public AtlasStandPrepSetpoints(AtlasJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultAngles();
   }

   private void useDefaultAngles()
   {
      setSetpoint(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);
      setSetpoint(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0);

      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      setSetpoint(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setSetpoint(robotSide, LegJointName.HIP_YAW, robotSide.negateIfRightSide(0.0));
         setSetpoint(robotSide, LegJointName.HIP_ROLL, robotSide.negateIfRightSide(0.062));
         setSetpoint(robotSide, LegJointName.HIP_PITCH, -0.233);
         setSetpoint(robotSide, LegJointName.KNEE_PITCH, 0.518);
         setSetpoint(robotSide, LegJointName.ANKLE_PITCH, -0.276);
         setSetpoint(robotSide, LegJointName.ANKLE_ROLL, robotSide.negateIfRightSide(-0.062));

         setSetpoint(robotSide, ArmJointName.SHOULDER_YAW, robotSide.negateIfRightSide(0.785398));
         setSetpoint(robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(-0.52379));
         setSetpoint(robotSide, ArmJointName.ELBOW_PITCH, 2.33708);
         setSetpoint(robotSide, ArmJointName.ELBOW_ROLL, robotSide.negateIfRightSide(2.35619));
         setSetpoint(robotSide, ArmJointName.FIRST_WRIST_PITCH, -0.337807);
         setSetpoint(robotSide, ArmJointName.WRIST_ROLL, robotSide.negateIfRightSide(0.20773));
         setSetpoint(robotSide, ArmJointName.SECOND_WRIST_PITCH, -0.026599);
      }
   }

   private void setSetpoint(RobotSide robotSide, LegJointName legJointName, double value)
   {
      setSetpoint(jointMap.getLegJointName(robotSide, legJointName), value);
   }

   private void setSetpoint(RobotSide robotSide, ArmJointName armJointName, double value)
   {
      setSetpoint(jointMap.getArmJointName(robotSide, armJointName), value);
   }

   private void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
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
