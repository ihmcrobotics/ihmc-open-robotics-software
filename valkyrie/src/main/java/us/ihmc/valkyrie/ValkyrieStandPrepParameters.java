package us.ihmc.valkyrie;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

public class ValkyrieStandPrepParameters implements StandPrepParameters
{
   private final HashMap<String, Double> setPoints = new HashMap<>();
   private final ValkyrieJointMap jointMap;

   public ValkyrieStandPrepParameters(ValkyrieJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultAngles();
      loadCustomSetPoints();
   }

   private void loadCustomSetPoints()
   {
      Yaml yaml = new Yaml();
      InputStream setpointsStream = ValkyrieStandPrepParameters.class.getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
      @SuppressWarnings("unchecked")
      Map<String, Double> setPointMap = (Map<String, Double>) yaml.load(setpointsStream);

      for (String jointName : ValkyrieOrderedJointMap.jointNames)
      {
         if (setPointMap.containsKey(jointName))
            setSetpoint(jointName, setPointMap.get(jointName));
      }

      try
      {
         setpointsStream.close();
      }
      catch (IOException e)
      {
      }
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
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 0.0);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.6);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.3);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.65);
         setSetpoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 0.0);

         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), -0.2);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfLeftSide(1.2));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), robotSide.negateIfLeftSide(1.5));
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 1.3);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0);
         setSetpoint(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
      }
   }

   private void setSetpoint(String jointName, double value)
   {
      setPoints.put(jointName, value);
   }

   @Override
   public double getSetpoint(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getSetpoint(jointName);
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
