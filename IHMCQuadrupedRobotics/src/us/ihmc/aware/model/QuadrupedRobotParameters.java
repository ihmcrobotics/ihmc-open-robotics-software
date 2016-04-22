package us.ihmc.aware.model;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.aware.mechanics.virtualModelControl.QuadrupedJointLimits;
import us.ihmc.quadrupedRobotics.parameters.*;

public abstract class QuadrupedRobotParameters
{
   private final QuadrupedContactPointParameters contactPointParameters = new QuadrupedContactPointParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract SDFFullQuadrupedRobotModel createFullRobotModel();

   public abstract String getModelName();

   public abstract QuadrupedPhysicalProperties getPhysicalProperties();

   public abstract QuadrupedActuatorParameters getActuatorParameters();

   public abstract QuadrupedSensorInformation getQuadrupedSensorInformation();

   public QuadrupedContactPointParameters getQuadrupedContactPointParameters()
   {
      return contactPointParameters;
   }
}
