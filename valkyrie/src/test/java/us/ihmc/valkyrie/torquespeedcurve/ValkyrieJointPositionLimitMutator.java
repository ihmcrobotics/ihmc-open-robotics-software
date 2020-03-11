package us.ihmc.valkyrie.torquespeedcurve;

import java.util.ArrayList;
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

public class ValkyrieJointPositionLimitMutator implements SDFDescriptionMutator
{
   private final Map<String, ArrayList<Double>> customJointPositionLimitMap = new HashMap<>();

   public ValkyrieJointPositionLimitMutator(ValkyrieJointMap jointMap, String jointName, ArrayList<Double> limits)
   {
	   System.out.printf("Adding mutator joint position limits of %s to [%f, %f]\n", jointName, limits.get(0), limits.get(1));
	   customJointPositionLimitMap.put(jointName, limits);
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if (customJointPositionLimitMap.containsKey(jointHolder.getName()))
      {
    	  ArrayList<Double> limits = customJointPositionLimitMap.get(jointHolder.getName());
    	  double lowerLimit = Math.toRadians(limits.get(0).doubleValue());
    	  double upperLimit = Math.toRadians(limits.get(1).doubleValue());
    	  jointHolder.setLimits(lowerLimit, upperLimit);
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
