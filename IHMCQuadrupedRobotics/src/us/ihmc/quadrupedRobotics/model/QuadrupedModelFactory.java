package us.ihmc.quadrupedRobotics.model;

import java.util.Collection;

import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.QuadrupedJointName;

public abstract class QuadrupedModelFactory implements FullQuadrupedRobotModelFactory
{
   public abstract FloatingRootJointRobot createSdfRobot();

   @Override
   public abstract FullQuadrupedRobotModel createFullRobotModel();

   public abstract Collection<QuadrupedJointName> getQuadrupedJointNames();

   public abstract String getSDFNameForJointName(QuadrupedJointName quadrupedJointName);
}
