package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.TunableJointAccelerationIntegrationParameters;
import us.ihmc.robotics.controllers.PIDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.TunableJointDesiredBehavior;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterTools
{
   public static void extractJointGainMap(List<ImmutablePair<PIDGains, List<String>>> jointspaceGains, Map<String, YoPIDGains> jointGainMapToPack,
                                          YoVariableRegistry registry)
   {
      jointGainMapToPack.clear();
      for (ImmutablePair<PIDGains, List<String>> gainPair : jointspaceGains)
      {
         PIDGains gains = gainPair.getLeft();
         YoPIDGains yoGains = new YoPIDGains(gains.getName(), registry);
         yoGains.set(gains);
         yoGains.createDerivativeGainUpdater(true);

         for (String jointName : gainPair.getRight())
         {
            jointGainMapToPack.put(jointName, yoGains);
         }
      }
   }

   public static void extractAccelerationIntegrationParameterMap(String prefix, List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> parameterList,
                                                                 Map<String, JointAccelerationIntegrationParametersReadOnly> parameterMapToPack,
                                                                 YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      if (parameterList == null)
      {
         return;
      }

      for (ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>> parameterTripple : parameterList)
      {
         String name = parameterTripple.getLeft();
         JointAccelerationIntegrationParametersReadOnly defaultParameters = parameterTripple.getMiddle();
         TunableJointAccelerationIntegrationParameters yoParameters = new TunableJointAccelerationIntegrationParameters(name + prefix, registry, defaultParameters);

         for (String jointName : parameterTripple.getRight())
         {
            parameterMapToPack.put(jointName, yoParameters);
         }
      }
   }

   public static void extractJointBehaviorMap(String prefix, List<ImmutableTriple<String, JointDesiredBehaviorReadOnly, List<String>>> parameterList,
                                              Map<String, JointDesiredBehaviorReadOnly> parameterMapToPack, YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      if (parameterList == null)
      {
         return;
      }

      for (ImmutableTriple<String, JointDesiredBehaviorReadOnly, List<String>> parameterTripple : parameterList)
      {
         String name = parameterTripple.getLeft();
         JointDesiredBehaviorReadOnly defaultParameters = parameterTripple.getMiddle();
         JointDesiredBehaviorReadOnly tunableParameters = new TunableJointDesiredBehavior(name + prefix, defaultParameters, registry);

         for (String jointName : parameterTripple.getRight())
         {
            parameterMapToPack.put(jointName, tunableParameters);
         }
      }
   }

   public static void extractGainMap(String suffix, List<ImmutableTriple<String, PID3DGains, List<String>>> gains, Map<String, YoPID3DGains> yoGainsToPack, YoVariableRegistry registry)
   {
      yoGainsToPack.clear();
      for (ImmutableTriple<String, PID3DGains, List<String>> gainTriple : gains)
      {
         String gainName = gainTriple.getLeft() + suffix;
         PID3DGains gain = gainTriple.getMiddle();
         YoPID3DGains yoGains = new DefaultYoPID3DGains(gainName, gain, registry);

         for (String bodyName : gainTriple.getRight())
         {
            yoGainsToPack.put(bodyName, yoGains);
         }
      }
   }
}
