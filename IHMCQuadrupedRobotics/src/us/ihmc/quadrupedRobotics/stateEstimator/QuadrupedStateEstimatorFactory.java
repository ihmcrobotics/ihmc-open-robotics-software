package us.ihmc.quadrupedRobotics.stateEstimator;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class QuadrupedStateEstimatorFactory
{
   
   public static DRCKinematicsBasedStateEstimator createStateEstimator(QuadrupedRobotParameters robotParameters, double gravity, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         SDFFullRobotModel fullRobotModel, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      RigidBody elevator = fullRobotModel.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBody estimationLink = fullRobotModel.getPelvis();
      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);
      
      QuadrupedSensorInformation sensorInformation = robotParameters.getQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = null;//robotParameters.getStateEstimatorParameters();
      QuadrupedJointNameMap jointMap = robotParameters.getJointMap();
      QuadrupedPhysicalProperties physicalProperties = robotParameters.getPhysicalProperties();
      CommonQuadrupedReferenceFrames estimatorReferenceFrames = new QuadrupedReferenceFrames(fullRobotModel, jointMap, physicalProperties);
      
      Map<RigidBody, ContactablePlaneBody> feet = new HashMap<>();
      Map<RigidBody, FootSwitchInterface> footSwitches = new HashMap<>();
      RobotMotionStatusHolder robotMotionStatusFromController = new RobotMotionStatusHolder();
      ForceSensorDataHolder forceSensorDataHolderToUpdate = null;

      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityMagnitude;

      ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();
      contactableBodiesFactory.createFootContactableBodiesMap(null, null)
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String namePrefix = quadrupedFeet.get(robotQuadrant).getName() + "StateEstimator";
         KinematicsBasedFootSwitch footSwitch = new KinematicsBasedFootSwitch(namePrefix, quadrupedFeet, switchZThreshold, totalRobotWeight, robotQuadrant, registry);
         footSwitches.put(robotQuadrant, footSwitch);

      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getImuNames();

      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters,
            sensorOutputMapReadOnly, forceSensorDataHolderToUpdate, imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitches, null,
            robotMotionStatusFromController, feet, yoGraphicsListRegistry);

      registry.addChild(stateEstimator.getYoVariableRegistry());

      return stateEstimator;
   }
   
   public static Map<RigidBody, ContactablePlaneBody> createFootContactableBodiesMap(SDFFullRobotModel fullRobotModel, QuadrupedJointNameMap jointMap, CommonQuadrupedReferenceFrames referenceFrames)
   {
      if (footContactPoints == null)
         return null;

      Map<RigidBody, ContactablePlaneBody> footContactableBodies = new LinkedHashMap<RigidBody, ContactablePlaneBody>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotQuadrant);
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, referenceFrames.getFootFrame(robotQuadrant),
               footContactPoints.get(robotSide));
         footContactableBodies.put(footContactableBody.getRigidBody(), footContactableBody);
      }
      return footContactableBodies;
   }
   
}
