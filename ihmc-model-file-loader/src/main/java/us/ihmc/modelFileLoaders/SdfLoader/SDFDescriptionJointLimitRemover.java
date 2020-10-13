package us.ihmc.modelFileLoaders.SdfLoader;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;

public class SDFDescriptionJointLimitRemover implements SDFDescriptionMutator
{
   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      jointHolder.setLimits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
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