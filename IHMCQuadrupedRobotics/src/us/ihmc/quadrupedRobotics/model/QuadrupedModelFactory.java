package us.ihmc.quadrupedRobotics.model;

import java.util.Collection;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModelFactory;
import us.ihmc.SdfLoader.FloatingRootJointRobot;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;

public abstract class QuadrupedModelFactory implements SDFFullQuadrupedRobotModelFactory
{
   public abstract FloatingRootJointRobot createSdfRobot();
   
   @Override
   public abstract SDFFullQuadrupedRobotModel createFullRobotModel();
   
   @Override
   public abstract GeneralizedSDFRobotModel getGeneralizedRobotModel();
   
   public abstract Collection<QuadrupedJointName> getQuadrupedJointNames();
   
   public abstract String getSDFNameForJointName(QuadrupedJointName quadrupedJointName);
}
