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

public class ValkyrieJointVelocityLimitMutator implements SDFDescriptionMutator
{
   private final Map<String, Double> customJointVelocityLimitMap = new HashMap<>();

   public ValkyrieJointVelocityLimitMutator(ValkyrieJointMap jointMap, String jointName, double velocityLimit)
   {
	   System.out.printf("Adding mutator velocity limit of %s to %f\n", jointName, velocityLimit);
	   customJointVelocityLimitMap.put(jointName, velocityLimit);
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if (customJointVelocityLimitMap.containsKey(jointHolder.getName()))
      {
         jointHolder.setVelocityLimit(customJointVelocityLimitMap.get(jointHolder.getName()));
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
