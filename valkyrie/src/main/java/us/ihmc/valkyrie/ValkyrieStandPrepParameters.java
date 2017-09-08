package us.ihmc.valkyrie;

import org.yaml.snakeyaml.Yaml;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepParameters;
import us.ihmc.valkyrie.parameters.ValkyrieOrderedJointMap;

import java.io.*;
import java.util.HashMap;
import java.util.Map;

public class ValkyrieStandPrepParameters implements StandPrepParameters
{
   private static HashMap<String, Double> setPoints = new HashMap<>();

   static
   {
      useDefaultAngles();
      loadCustomSetPoints();
   }

   private static void loadCustomSetPoints()
   {
      Yaml yaml = new Yaml();
      InputStream setpointsStream = ValkyrieStandPrepParameters.class.getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
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

   private static void useDefaultAngles()
   {                                      
      setSetpoint(ValkyrieOrderedJointMap.LeftHipYaw         ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.LeftHipRoll        ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.LeftHipPitch       , -0.6);
      setSetpoint(ValkyrieOrderedJointMap.LeftKneePitch      ,  1.3);
      setSetpoint(ValkyrieOrderedJointMap.LeftAnklePitch     ,  -0.65);
      setSetpoint(ValkyrieOrderedJointMap.LeftAnkleRoll      ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.RightHipYaw        ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.RightHipRoll       ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.RightHipPitch      , -0.6);
      setSetpoint(ValkyrieOrderedJointMap.RightKneePitch     ,  1.3);
      setSetpoint(ValkyrieOrderedJointMap.RightAnklePitch    ,  -0.65);
      setSetpoint(ValkyrieOrderedJointMap.RightAnkleRoll     ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.TorsoYaw           ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.TorsoPitch         ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.TorsoRoll          ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.LeftShoulderPitch  , -0.2);
      setSetpoint(ValkyrieOrderedJointMap.LeftShoulderRoll   , -1.2);
      setSetpoint(ValkyrieOrderedJointMap.LeftShoulderYaw    ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.LeftElbowPitch     , -1.5);
      setSetpoint(ValkyrieOrderedJointMap.LeftForearmYaw     ,  1.3);
      setSetpoint(ValkyrieOrderedJointMap.LeftWristRoll      ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.LeftWristPitch     ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.LowerNeckPitch     ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.NeckYaw            ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.UpperNeckPitch     ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.RightShoulderPitch , -0.2);
      setSetpoint(ValkyrieOrderedJointMap.RightShoulderRoll  ,  1.2);
      setSetpoint(ValkyrieOrderedJointMap.RightShoulderYaw   ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.RightElbowPitch    ,  1.5);
      setSetpoint(ValkyrieOrderedJointMap.RightForearmYaw    ,  1.3);
      setSetpoint(ValkyrieOrderedJointMap.RightWristRoll     ,  0.0);
      setSetpoint(ValkyrieOrderedJointMap.RightWristPitch    ,  0.0);
   }


   private static void setSetpoint(int jointIndex, double value)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      setSetpoint(jointName, value);
   }

   private static void setSetpoint(String jointName, double value)
   {
      if(setPoints.containsKey(jointName))
      {
         setPoints.remove(jointName);
      }
      setPoints.put(jointName, value);
   }

   public double getSetpoint(int jointIndex)
   {
      String jointName = ValkyrieOrderedJointMap.jointNames[jointIndex];
      return getSetpoint(jointName);
   }

   public double getSetpoint(String jointName)
   {
      return setPoints.get(jointName);
   }
}
