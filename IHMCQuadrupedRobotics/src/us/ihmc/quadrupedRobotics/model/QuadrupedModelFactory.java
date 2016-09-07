package us.ihmc.quadrupedRobotics.model;

import java.util.Collection;

import us.ihmc.SdfLoader.FloatingRootJointRobot;
import us.ihmc.SdfLoader.FullQuadrupedRobotModelFactory;
import us.ihmc.SdfLoader.models.FullQuadrupedRobotModel;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;

public abstract class QuadrupedModelFactory implements FullQuadrupedRobotModelFactory
{
   public abstract FloatingRootJointRobot createSdfRobot();

   @Override
   public abstract FullQuadrupedRobotModel createFullRobotModel();

   public abstract Collection<QuadrupedJointName> getQuadrupedJointNames();

   public abstract String getSDFNameForJointName(QuadrupedJointName quadrupedJointName);
}
