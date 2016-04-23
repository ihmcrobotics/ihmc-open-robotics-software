package us.ihmc.aware.model;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFRobot;

public abstract class QuadrupedRobotParameters
{
   private final QuadrupedContactPointParameters contactPointParameters = new QuadrupedContactPointParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract SDFFullQuadrupedRobotModel createFullRobotModel();

   public abstract QuadrupedPhysicalProperties getPhysicalProperties();

   public QuadrupedContactPointParameters getQuadrupedContactPointParameters()
   {
      return contactPointParameters;
   }
}
