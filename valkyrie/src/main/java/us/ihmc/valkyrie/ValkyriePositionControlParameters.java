package us.ihmc.valkyrie;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionControlParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

public class ValkyriePositionControlParameters implements PositionControlParameters
{
   private final HashMap<String, Double> proportionalGain = new HashMap<>();
   private final HashMap<String, Double> derivativeGain = new HashMap<>();
   private final HashMap<String, Double> integralGain = new HashMap<>();
   private final ValkyrieJointMap jointMap;

   public ValkyriePositionControlParameters(ValkyrieJointMap jointMap)
   {
      this.jointMap = jointMap;
      useDefaultGains();
      loadCustomGains();
   }

   @SuppressWarnings("unchecked")
   private void loadCustomGains()
   {
      Yaml yaml = new Yaml();
      InputStream gainStream = ValkyriePositionControlParameters.class.getClassLoader().getResourceAsStream("standPrep/gains.yaml");

      Map<String, Map<String, Double>> gainMap = (Map<String, Map<String, Double>>) yaml.load(gainStream);

      try
      {
         gainStream.close();
      }
      catch (IOException e)
      {
         return;
      }

      for (String jointName : ValkyrieOrderedJointMap.jointNames)
      {
         Map<String, Double> jointGains = gainMap.get(jointName);
         if (jointGains == null)
         {
            setJointGains(jointName, 0.0, 0.0, 0.0);
         }
         else
         {
            Double kp = jointGains.get("kp");
            Double ki = jointGains.get("ki");
            Double kd = jointGains.get("kd");
            setProportionalGain(jointName, kp == null ? 0.0 : kp);
            setIntegralGain(jointName, ki == null ? 0.0 : ki);
            setDerivativeGain(jointName, kd == null ? 0.0 : kd);
         }
      }
   }

   private void useDefaultGains()
   {
      setJointGains(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0, 0.0, 0.0);
      setJointGains(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0, 0.0, 0.0);
      setJointGains(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.0, 0.0, 0.0);

      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 100.0, 0.0, 6.2);
      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 600.0, 0.0, 40.0);
      setJointGains(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 600.0, 0.0, 40.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 50.0, 0.0, 5.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), 50.0, 0.0, 5.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 40.0, 0.0, 4.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 40.0, 0.0, 10.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), 0.0, 0.0, 0.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), 0.0, 0.0, 0.0);
         setJointGains(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0, 0.0, 0.0);

         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 100.0, 0.0, 6.2);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), 500.0, 0.0, 25.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), 550.0, 0.0, 21.0);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 259.0, 0.0, 9.4);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), 68.0, 0.0, 6.29);
         setJointGains(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), 56.0, 0.0, 3.14);
      }
   }

   private void setJointGains(String jointName, double kp, double ki, double kd)
   {
      setProportionalGain(jointName, kp);
      setIntegralGain(jointName, ki);
      setDerivativeGain(jointName, kd);
   }

   private void setProportionalGain(String jointName, double value)
   {
      if (proportionalGain.containsKey(jointName))
      {
         proportionalGain.remove(jointName);
      }
      proportionalGain.put(jointName, value);
   }

   private void setDerivativeGain(String jointName, double value)
   {
      if (derivativeGain.containsKey(jointName))
      {
         derivativeGain.remove(jointName);
      }
      derivativeGain.put(jointName, value);
   }

   private void setIntegralGain(String jointName, double value)
   {
      if (integralGain.containsKey(jointName))
      {
         integralGain.remove(jointName);
      }
      integralGain.put(jointName, value);
   }

   @Override
   public double getProportionalGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getProportionalGain(jointName);
   }

   @Override
   public double getProportionalGain(String jointName)
   {
      return proportionalGain.get(jointName);
   }

   @Override
   public double getDerivativeGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getDerivativeGain(jointName);
   }

   @Override
   public double getDerivativeGain(String jointName)
   {
      return derivativeGain.get(jointName);
   }

   @Override
   public double getIntegralGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getIntegralGain(jointName);
   }

   @Override
   public double getIntegralGain(String jointName)
   {
      return integralGain.get(jointName);
   }

   @Override
   public double getPositionControlMasterGain()
   {
      return 1.0;
   }
}
