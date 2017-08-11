package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.YoJointAccelerationIntegrationParameters;
import us.ihmc.robotics.controllers.PIDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
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

   public static void extractAccelerationIntegrationParameterMap(List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> parameterList,
                                                                  Map<String, JointAccelerationIntegrationParametersReadOnly> parameterMapToPack,
                                                                  YoVariableRegistry registry)
   {
      parameterMapToPack.clear();
      for (ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>> parameterTripple : parameterList)
      {
         String name = parameterTripple.getLeft();
         YoJointAccelerationIntegrationParameters yoParameters = new YoJointAccelerationIntegrationParameters(name, registry);
         yoParameters.set(parameterTripple.getMiddle());

         for (String jointName : parameterTripple.getRight())
         {
            parameterMapToPack.put(jointName, yoParameters);
         }
      }
   }
}
