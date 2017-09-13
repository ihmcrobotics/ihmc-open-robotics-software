package us.ihmc.valkyrie;

import org.yaml.snakeyaml.Yaml;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.PositionControlParameters;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

public class ValkyriePositionControlParameters implements PositionControlParameters
{
   private static HashMap<String, Double> proportionalGain = new HashMap<>();
   private static HashMap<String, Double> derivativeGain = new HashMap<>();
   private static HashMap<String, Double> integralGain = new HashMap<>();

   static
   {
      useDefaultGains();
      loadCustomGains();
   }

   private static void loadCustomGains()
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
      }
   }

   private static void useDefaultGains()
   {
      setProportionalGain(ValkyrieOrderedJointMap.LeftHipYaw,          100.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftHipRoll,         500.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftHipPitch,        550.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftKneePitch,       259.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftAnklePitch,       68.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftAnkleRoll,        56.0);

      setProportionalGain(ValkyrieOrderedJointMap.RightHipYaw,         100.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightHipRoll,        500.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightHipPitch,       550.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightKneePitch,      259.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightAnklePitch,      68.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightAnkleRoll,       56.0);

      setProportionalGain(ValkyrieOrderedJointMap.TorsoYaw,            100.0);
      setProportionalGain(ValkyrieOrderedJointMap.TorsoPitch,          600.0);
      setProportionalGain(ValkyrieOrderedJointMap.TorsoRoll,           600.0);

      setProportionalGain(ValkyrieOrderedJointMap.LeftShoulderPitch,    50.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftShoulderRoll,     50.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftShoulderYaw,      40.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftElbowPitch,       40.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftForearmYaw,        0.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftWristRoll,         0.0);
      setProportionalGain(ValkyrieOrderedJointMap.LeftWristPitch,        0.0);

      setProportionalGain(ValkyrieOrderedJointMap.LowerNeckPitch,        0.0);
      setProportionalGain(ValkyrieOrderedJointMap.NeckYaw,               0.0);
      setProportionalGain(ValkyrieOrderedJointMap.UpperNeckPitch,        0.0);

      setProportionalGain(ValkyrieOrderedJointMap.RightShoulderPitch,   50.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightShoulderRoll,    50.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightShoulderYaw,     40.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightElbowPitch,      40.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightForearmYaw,       0.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightWristRoll,        0.0);
      setProportionalGain(ValkyrieOrderedJointMap.RightWristPitch,       0.0);

      setDerivativeGain(ValkyrieOrderedJointMap.LeftHipYaw,            6.2);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftHipRoll,          25.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftHipPitch,         21.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftKneePitch,         9.4);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftAnklePitch,        6.29);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftAnkleRoll,         3.14);

      setDerivativeGain(ValkyrieOrderedJointMap.RightHipYaw,           6.2);
      setDerivativeGain(ValkyrieOrderedJointMap.RightHipRoll,         25.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightHipPitch,        21.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightKneePitch,        9.4);
      setDerivativeGain(ValkyrieOrderedJointMap.RightAnklePitch,       6.29);
      setDerivativeGain(ValkyrieOrderedJointMap.RightAnkleRoll,        3.14);

      setDerivativeGain(ValkyrieOrderedJointMap.TorsoYaw,              6.2);
      setDerivativeGain(ValkyrieOrderedJointMap.TorsoPitch,           40.0);
      setDerivativeGain(ValkyrieOrderedJointMap.TorsoRoll,            40.0);

      setDerivativeGain(ValkyrieOrderedJointMap.LeftShoulderPitch,     5.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftShoulderRoll,      5.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftShoulderYaw,       4.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftElbowPitch,       10.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftForearmYaw,        0.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftWristRoll,         0.0);
      setDerivativeGain(ValkyrieOrderedJointMap.LeftWristPitch,        0.0);

      setDerivativeGain(ValkyrieOrderedJointMap.LowerNeckPitch,        0.0);
      setDerivativeGain(ValkyrieOrderedJointMap.NeckYaw,               0.0);
      setDerivativeGain(ValkyrieOrderedJointMap.UpperNeckPitch,        0.0);

      setDerivativeGain(ValkyrieOrderedJointMap.RightShoulderPitch,    5.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightShoulderRoll,     5.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightShoulderYaw,      4.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightElbowPitch,       4.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightForearmYaw,       0.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightWristRoll,        0.0);
      setDerivativeGain(ValkyrieOrderedJointMap.RightWristPitch,       0.0);

      setIntegralGain(ValkyrieOrderedJointMap.LeftHipYaw,          0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftHipRoll,         0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftHipPitch,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftKneePitch,       0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftAnklePitch,      0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftAnkleRoll,       0.0);

      setIntegralGain(ValkyrieOrderedJointMap.RightHipYaw,         0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightHipRoll,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightHipPitch,       0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightKneePitch,      0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightAnklePitch,     0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightAnkleRoll,      0.0);

      setIntegralGain(ValkyrieOrderedJointMap.TorsoYaw,            0.0);
      setIntegralGain(ValkyrieOrderedJointMap.TorsoPitch,          0.0);
      setIntegralGain(ValkyrieOrderedJointMap.TorsoRoll,           0.0);

      setIntegralGain(ValkyrieOrderedJointMap.LeftShoulderPitch,     0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftShoulderRoll,      0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftShoulderYaw,       0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftElbowPitch,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftForearmYaw,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftWristRoll,         0.0);
      setIntegralGain(ValkyrieOrderedJointMap.LeftWristPitch,        0.0);

      setIntegralGain(ValkyrieOrderedJointMap.LowerNeckPitch,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.NeckYaw,               0.0);
      setIntegralGain(ValkyrieOrderedJointMap.UpperNeckPitch,        0.0);

      setIntegralGain(ValkyrieOrderedJointMap.RightShoulderPitch,     0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightShoulderRoll,      0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightShoulderYaw,       0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightElbowPitch,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightForearmYaw,        0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightWristRoll,         0.0);
      setIntegralGain(ValkyrieOrderedJointMap.RightWristPitch,        0.0);
   }

   
   private static void setProportionalGain(int jointIndex, double value)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      setProportionalGain(jointName, value);
   }

   private static void setProportionalGain(String jointName, double value)
   {
      if(proportionalGain.containsKey(jointName))
      {
         proportionalGain.remove(jointName);
      }
      proportionalGain.put(jointName, value);
   }

   private static void setDerivativeGain(int jointIndex, double value)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      setDerivativeGain(jointName, value);
   }

   private static void setDerivativeGain(String jointName, double value)
   {
      if(derivativeGain.containsKey(jointName))
      {
         derivativeGain.remove(jointName);
      }
      derivativeGain.put(jointName, value);
   }

   private static void setIntegralGain(int jointIndex, double value)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      setIntegralGain(jointName, value);
   }

   private static void setIntegralGain(String jointName, double value)
   {
      if(integralGain.containsKey(jointName))
      {
         integralGain.remove(jointName);
      }
      integralGain.put(jointName, value);
   }

   public double getProportionalGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getProportionalGain(jointName);
   }

   public double getProportionalGain(String jointName)
   {
      return proportionalGain.get(jointName);
   }

   public double getDerivativeGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getDerivativeGain(jointName);
   }

   public double getDerivativeGain(String jointName)
   {
      return derivativeGain.get(jointName);
   }

   public double getIntegralGain(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getIntegralGain(jointName);
   }

   public double getIntegralGain(String jointName)
   {
      return integralGain.get(jointName);
   }

   public double getPositionControlMasterGain()
   {
      return 3.0;
   }
}
