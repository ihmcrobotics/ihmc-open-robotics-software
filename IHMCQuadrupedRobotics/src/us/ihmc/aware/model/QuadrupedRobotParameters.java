package us.ihmc.aware.model;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.aware.mechanics.virtualModelControl.QuadrupedJointLimits;
import us.ihmc.quadrupedRobotics.parameters.*;

public abstract class QuadrupedRobotParameters
{
   private final QuadrupedJointLimits jointLimits = new QuadrupedJointLimits();
   private final QuadrupedContactPointParameters contactPointParameters = new QuadrupedContactPointParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract SDFFullQuadrupedRobotModel createFullRobotModel();

   public abstract String getModelName();

   public abstract SDFParameters getSdfParameters();

   public abstract QuadrupedPhysicalProperties getPhysicalProperties();

   public abstract QuadrupedActuatorParameters getActuatorParameters();

   public abstract QuadrupedInitialPositionParameters getQuadrupedInitialPositionParameters();

   public abstract QuadrupedPositionBasedCrawlControllerParameters getQuadrupedPositionBasedCrawlControllerParameters();
   
   public abstract QuadrupedSensorInformation getQuadrupedSensorInformation();

   public QuadrupedJointLimits getQuadrupedJointLimits()
   {
      return jointLimits;
   }
   
   public QuadrupedContactPointParameters getQuadrupedContactPointParameters()
   {
      return contactPointParameters;
   }

}
