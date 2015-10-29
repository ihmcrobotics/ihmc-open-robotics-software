package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedDataProvider;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedPositionBasedCrawlControllerFactory
{
   private final QuadrupedRobotParameters robotParameters;
   private final SDFRobot sdfRobot;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final QuadrupedJointNameMap quadrupedJointNameMap;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private QuadrupedDataProvider dataProvider;

   public QuadrupedPositionBasedCrawlControllerFactory(QuadrupedRobotParameters robotParameters, SDFRobot sdfRobot, SDFFullRobotModel sdfFullRobotModel,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotParameters = robotParameters;
      this.sdfRobot = sdfRobot;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.quadrupedJointNameMap = robotParameters.getJointMap();
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      //create GlobalDataP
   }

   public QuadrupedPositionBasedCrawlController createCrawlController(double simulationDt, QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculator)
   {
      QuadrupedPhysicalProperties physicalProperties = robotParameters.getPhysicalProperties();
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(sdfFullRobotModel, quadrupedJointNameMap, physicalProperties);

      DoubleYoVariable yoTime = sdfRobot.getYoTime();

      return new QuadrupedPositionBasedCrawlController(simulationDt, robotParameters, sdfFullRobotModel, quadrupedJointNameMap, referenceFrames,
            inverseKinematicsCalculator, yoGraphicsListRegistry, yoTime, dataProvider.getDesiredVelocityProvider(), dataProvider.getDesiredYawRateProvider());
   }
}
