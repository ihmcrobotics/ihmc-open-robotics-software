package us.ihmc.modelFileLoaders.SdfLoader;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;

public class SDFDescriptionJointLimitRemover implements SDFDescriptionMutator
{
   // Using MAX_VALUE or INFINITY can throw off the QP
   private static final double MIN_MAX_LIMIT = 100.0;

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      jointHolder.setLimits(-MIN_MAX_LIMIT, MIN_MAX_LIMIT);
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