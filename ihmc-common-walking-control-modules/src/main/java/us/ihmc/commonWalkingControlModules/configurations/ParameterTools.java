package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.TunableJointAccelerationIntegrationParameters;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.TunableJointDesiredBehavior;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterTools
{
   public static void extractJointGainMap(List<JointGroupParameter<PIDGainsReadOnly>> jointspaceGains, Map<String, PIDGainsReadOnly> jointGainMapToPack,
                                          YoVariableRegistry registry)
   {
      jointGainMapToPack.clear();
      for (JointGroupParameter<PIDGainsReadOnly> jointGroupParameter : jointspaceGains)
      {
         String name = jointGroupParameter.getJointGroupName();
         PIDGainsReadOnly parameter = jointGroupParameter.getParameter();
         ParameterizedPIDGains parameterizedGains = new ParameterizedPIDGains(name, parameter, registry);

         for (String jointName : jointGroupParameter.getJointNames())
         {
            jointGainMapToPack.put(jointName, parameterizedGains);
         }
      }
   }

   public static void extractAccelerationIntegrationParameterMap(String prefix, List<JointGroupParameter<JointAccelerationIntegrationParametersReadOnly>> parameterList,
                                                                 Map<String, JointAccelerationIntegrationParametersReadOnly> parameterMapToPack,
                                                                 YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      if (parameterList == null)
      {
         return;
      }

      for (JointGroupParameter<JointAccelerationIntegrationParametersReadOnly> jointGroupParameter : parameterList)
      {
         String name = jointGroupParameter.getJointGroupName();
         JointAccelerationIntegrationParametersReadOnly defaultParameters = jointGroupParameter.getParameter();
         TunableJointAccelerationIntegrationParameters parameterizedParameters = new TunableJointAccelerationIntegrationParameters(name + prefix, registry, defaultParameters);

         for (String jointName : jointGroupParameter.getJointNames())
         {
            parameterMapToPack.put(jointName, parameterizedParameters);
         }
      }
   }

   public static void extractJointBehaviorMap(String prefix, List<JointGroupParameter<JointDesiredBehaviorReadOnly>> parameterList,
                                              Map<String, JointDesiredBehaviorReadOnly> parameterMapToPack, YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      if (parameterList == null)
      {
         return;
      }

      for (JointGroupParameter<JointDesiredBehaviorReadOnly> jointGroupParameter : parameterList)
      {
         String name = jointGroupParameter.getJointGroupName();
         JointDesiredBehaviorReadOnly defaultParameters = jointGroupParameter.getParameter();
         JointDesiredBehaviorReadOnly tunableParameters = new TunableJointDesiredBehavior(prefix + name, defaultParameters, registry);

         for (String jointName : jointGroupParameter.getJointNames())
         {
            parameterMapToPack.put(jointName, tunableParameters);
         }
      }
   }

   public static void extract3DGainMap(String suffix, List<JointGroupParameter<PID3DGainsReadOnly>> gains, Map<String, PID3DGainsReadOnly> yoGainsToPack, YoVariableRegistry registry)
   {
      yoGainsToPack.clear();
      for (JointGroupParameter<PID3DGainsReadOnly> jointGroupGains : gains)
      {
         String gainName = jointGroupGains.getJointGroupName() + suffix;
         PID3DGainsReadOnly gain = jointGroupGains.getParameter();
         PID3DGainsReadOnly parameterizedGains = new ParameterizedPID3DGains(gainName, gain, registry);

         for (String bodyName : jointGroupGains.getJointNames())
         {
            yoGainsToPack.put(bodyName, parameterizedGains);
         }
      }
   }
}
