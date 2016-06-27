package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class QuadrupedStateEstimatorFactory
{

   private static double switchZThreshold = 0.005;
   
   public static DRCKinematicsBasedStateEstimator createStateEstimator(QuadrupedSensorInformation sensorInformation, StateEstimatorParameters stateEstimatorParameters, SDFFullRobotModel fullRobotModel, SensorOutputMapReadOnly sensorOutputMapReadOnly,
         QuadrantDependentList<ContactablePlaneBody> feet, QuadrantDependentList<FootSwitchInterface> footSwitches, double gravity, double estimatorDT, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      RigidBody elevator = fullRobotModel.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBody estimationLink = fullRobotModel.getPelvis();
      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      RobotMotionStatusHolder robotMotionStatusFromController = new RobotMotionStatusHolder();
      robotMotionStatusFromController.setCurrentRobotMotionStatus(RobotMotionStatus.IN_MOTION);
      ForceSensorDataHolder forceSensorDataHolderToUpdate = null;

      Map<RigidBody, ContactablePlaneBody> feetMap = new HashMap<RigidBody, ContactablePlaneBody>();
      Map<RigidBody, FootSwitchInterface> footSwitchMap = new HashMap<RigidBody, FootSwitchInterface>();
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactablePlaneBody = feet.get(quadrant);
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         feetMap.put(rigidBody, contactablePlaneBody);
         FootSwitchInterface footSwitch = footSwitches.get(quadrant);
         footSwitchMap.put(rigidBody, footSwitch);
      }
      
      String[] imuSensorsToUseInStateEstimator = sensorInformation.getImuNames();
      double gravityMagnitude = Math.abs(gravity);

      // Create the sensor readers and state estimator here:
      DRCKinematicsBasedStateEstimator stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters, sensorOutputMapReadOnly, forceSensorDataHolderToUpdate, imuSensorsToUseInStateEstimator,
            gravityMagnitude, footSwitchMap, null, robotMotionStatusFromController, feetMap, yoGraphicsListRegistry);

      registry.addChild(stateEstimator.getYoVariableRegistry());

      return stateEstimator;
   }

   public static QuadrantDependentList<FootSwitchInterface> createFootSwitches(QuadrantDependentList<ContactablePlaneBody> quadrupedFeet, double gravity, SDFFullRobotModel fullRobotModel, YoVariableRegistry registry)
   {
      QuadrantDependentList<FootSwitchInterface> footSwitches = new QuadrantDependentList<FootSwitchInterface>();
      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityMagnitude;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactablePlaneBody = quadrupedFeet.get(robotQuadrant);
         String namePrefix = contactablePlaneBody.getName() + "StateEstimator";
         SettableFootSwitch footSwitch = new SettableFootSwitch(quadrupedFeet.get(robotQuadrant), robotQuadrant, totalRobotWeight, registry);
//         KinematicsBasedFootSwitch footSwitch = new KinematicsBasedFootSwitch(namePrefix, quadrupedFeet, switchZThreshold, totalRobotWeight, robotQuadrant, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
      return footSwitches;
   }

   public static QuadrantDependentList<ContactablePlaneBody> createFootContactableBodies(SDFFullQuadrupedRobotModel fullRobotModel, CommonQuadrupedReferenceFrames referenceFrames,
         QuadrupedPhysicalProperties quadrupedPhysicalProperties)
   {
      QuadrantDependentList<ContactablePlaneBody> footContactableBodies = new QuadrantDependentList<ContactablePlaneBody>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotQuadrant);
         ListOfPointsContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, referenceFrames.getFootFrame(robotQuadrant), quadrupedPhysicalProperties.getFootGroundContactPoints(robotQuadrant));
         footContactableBodies.set(robotQuadrant, footContactableBody);
      }
      return footContactableBodies;
   }
}
