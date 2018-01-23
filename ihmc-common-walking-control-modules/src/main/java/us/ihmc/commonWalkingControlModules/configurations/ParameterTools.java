package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.TunableJointAccelerationIntegrationParameters;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPIDGains;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.TunableJointDesiredBehavior;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterTools
{
   public static void extractJointGainMap(List<GroupParameter<PIDGainsReadOnly>> jointspaceGains, Map<String, PIDGainsReadOnly> jointGainMapToPack,
                                          YoVariableRegistry registry)
   {
      jointGainMapToPack.clear();
      for (GroupParameter<PIDGainsReadOnly> jointGroupParameter : jointspaceGains)
      {
         String name = jointGroupParameter.getGroupName();
         PIDGainsReadOnly parameter = jointGroupParameter.getParameter();
         ParameterizedPIDGains parameterizedGains = new ParameterizedPIDGains(name, parameter, registry);

         for (String jointName : jointGroupParameter.getMemberNames())
         {
            jointGainMapToPack.put(jointName, parameterizedGains);
         }
      }
   }

   public static void extractAccelerationIntegrationParameterMap(String prefix, List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> parameterList,
                                                                 Map<String, JointAccelerationIntegrationParametersReadOnly> parameterMapToPack,
                                                                 YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      if (parameterList == null)
      {
         return;
      }

      for (GroupParameter<JointAccelerationIntegrationParametersReadOnly> jointGroupParameter : parameterList)
      {
         String name = jointGroupParameter.getGroupName();
         JointAccelerationIntegrationParametersReadOnly defaultParameters = jointGroupParameter.getParameter();
         TunableJointAccelerationIntegrationParameters parameterizedParameters = new TunableJointAccelerationIntegrationParameters(prefix + name, registry, defaultParameters);

         for (String jointName : jointGroupParameter.getMemberNames())
         {
            parameterMapToPack.put(jointName, parameterizedParameters);
         }
      }
   }

   public static void extractJointBehaviorMap(String prefix, List<GroupParameter<JointDesiredBehaviorReadOnly>> parameterList,
                                              Map<String, JointDesiredBehaviorReadOnly> parameterMapToPack, YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      if (parameterList == null)
      {
         return;
      }

      for (GroupParameter<JointDesiredBehaviorReadOnly> jointGroupParameter : parameterList)
      {
         String name = jointGroupParameter.getGroupName();
         JointDesiredBehaviorReadOnly defaultParameters = jointGroupParameter.getParameter();
         JointDesiredBehaviorReadOnly tunableParameters = new TunableJointDesiredBehavior(prefix + name, defaultParameters, registry);

         for (String jointName : jointGroupParameter.getMemberNames())
         {
            parameterMapToPack.put(jointName, tunableParameters);
         }
      }
   }

   public static void extract3DGainMap(String suffix, List<GroupParameter<PID3DGainsReadOnly>> gains, Map<String, PID3DGainsReadOnly> yoGainsToPack, YoVariableRegistry registry)
   {
      yoGainsToPack.clear();
      for (GroupParameter<PID3DGainsReadOnly> jointGroupGains : gains)
      {
         String gainName = jointGroupGains.getGroupName() + suffix;
         PID3DGainsReadOnly gain = jointGroupGains.getParameter();
         PID3DGainsReadOnly parameterizedGains = new ParameterizedPID3DGains(gainName, gain, registry);

         for (String bodyName : jointGroupGains.getMemberNames())
         {
            yoGainsToPack.put(bodyName, parameterizedGains);
         }
      }
   }

   public static void extractJointWeightMap(String suffix, List<GroupParameter<Double>> jointspaceWeights, Map<String, DoubleProvider> jointWeightMapToPack,
                                            YoVariableRegistry registry)
   {
      jointWeightMapToPack.clear();
      for (GroupParameter<Double> jointGroupParameter : jointspaceWeights)
      {
         String name = jointGroupParameter.getGroupName() + suffix;
         Double defaultWeight = jointGroupParameter.getParameter();
         DoubleParameter tunableWeight = new DoubleParameter(name, registry, defaultWeight.doubleValue());

         for (String jointName : jointGroupParameter.getMemberNames())
         {
            jointWeightMapToPack.put(jointName, tunableWeight);
         }
      }
   }

   public static void extract3DWeightMap(String suffix, List<GroupParameter<Vector3DReadOnly>> weights, Map<String, Vector3DReadOnly> weightsToPack,
                                         YoVariableRegistry registry)
   {
      weightsToPack.clear();
      for (GroupParameter<Vector3DReadOnly> jointGroupWeights : weights)
      {
         String gainName = jointGroupWeights.getGroupName() + suffix;
         Vector3DReadOnly weight = jointGroupWeights.getParameter();
         Vector3DReadOnly parameterizedWeights = new ParameterVector3D(gainName, weight, registry);

         for (String bodyName : jointGroupWeights.getMemberNames())
         {
            weightsToPack.put(bodyName, parameterizedWeights);
         }
      }
   }
}
