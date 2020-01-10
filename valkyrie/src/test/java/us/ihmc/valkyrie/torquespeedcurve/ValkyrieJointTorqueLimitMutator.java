package us.ihmc.valkyrie.torquespeedcurve;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieJointTorqueLimitMutator implements SDFDescriptionMutator
{
   private final Map<String, Double> customJointTorqueLimitMap = new HashMap<>();

   public ValkyrieJointTorqueLimitMutator(ValkyrieJointMap jointMap)
   {
      // Example how to setup custom joint torque limit:
      for (RobotSide robotSide : RobotSide.values)
      {
         customJointTorqueLimitMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 350.0);
      }
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if (customJointTorqueLimitMap.containsKey(jointHolder.getName()))
      {
         jointHolder.setEffortLimit(customJointTorqueLimitMap.get(jointHolder.getName()));
      }
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
   }
}
