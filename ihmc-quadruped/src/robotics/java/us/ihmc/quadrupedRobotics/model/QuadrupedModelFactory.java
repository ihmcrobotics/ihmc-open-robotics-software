package us.ihmc.quadrupedRobotics.model;

import java.io.InputStream;
import java.util.Collection;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.partNames.QuadrupedJointNameMap;

public abstract class QuadrupedModelFactory implements FullQuadrupedRobotModelFactory
{
   @Override
   public abstract FullQuadrupedRobotModel createFullRobotModel();

   public abstract QuadrupedJointNameMap getJointMap();

   public abstract Collection<QuadrupedJointName> getQuadrupedJointNames();

   public abstract String getSDFNameForJointName(QuadrupedJointName quadrupedJointName);

   public abstract String getParameterResourceName();

   public abstract InputStream getParameterInputStream();
}
