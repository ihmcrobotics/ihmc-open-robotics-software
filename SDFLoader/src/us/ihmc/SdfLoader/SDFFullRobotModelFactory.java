package us.ihmc.SdfLoader;

import us.ihmc.utilities.humanoidRobot.model.FullRobotModelFactory;

public class SDFFullRobotModelFactory implements FullRobotModelFactory
{
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;
   private final SDFJointNameMap sdfJointNameMap;
   
   public SDFFullRobotModelFactory(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap)
   {
      this.generalizedSDFRobotModel = generalizedSDFRobotModel;
      this.sdfJointNameMap = sdfJointNameMap;
   }

   public SDFFullRobotModel createFullRobotModel()
   {
      return new SDFFullRobotModel(generalizedSDFRobotModel.getRootLinks().get(0), sdfJointNameMap, new String[0]);
   }
}
