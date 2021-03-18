package us.ihmc.quadrupedRobotics.estimator.stateEstimator;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

public class QuadrupedStateEstimatorFactory
{
   private final RequiredFactoryField<QuadrupedSensorInformation> sensorInformation = new RequiredFactoryField<>("sensorInformation");
   private final RequiredFactoryField<StateEstimatorParameters> stateEstimatorParameters = new RequiredFactoryField<>("stateEstimatorParameters");
   private final RequiredFactoryField<FullRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<SensorOutputMapReadOnly> sensorOutputMapReadOnly = new RequiredFactoryField<>("sensorOutputMapReadOnly");
   private final RequiredFactoryField<QuadrantDependentList<ContactablePlaneBody>> footContactableBodies = new RequiredFactoryField<>("footContactableBodies");
   private final RequiredFactoryField<QuadrantDependentList<FootSwitchInterface>> footSwitches = new RequiredFactoryField<>("footSwitches");
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<Double> estimatorDT = new RequiredFactoryField<>("estimatorDT");
   private final RequiredFactoryField<CenterOfMassDataHolder> centerOfMassDataHolder = new RequiredFactoryField<>("centerOfMassDataHolder");
   private final RequiredFactoryField<YoGraphicsListRegistry> yoGraphicsListRegistry = new RequiredFactoryField<>("yoGraphicsListRegistry");
   private final RequiredFactoryField<RobotMotionStatusHolder> robotMotionStatusFromController = new RequiredFactoryField<>("robotMotionStatusFromController");

   public StateEstimatorController createStateEstimator()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      RigidBodyBasics elevator = fullRobotModel.get().getElevator();
      FloatingJointBasics rootInverseDynamicsJoint = fullRobotModel.get().getRootJoint();
      RigidBodyBasics estimationLink = fullRobotModel.get().getRootBody();
      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      CenterOfPressureDataHolder centerOfPressureDataHolder = null;

      Map<RigidBodyBasics, ContactablePlaneBody> feetMap = new HashMap<RigidBodyBasics, ContactablePlaneBody>();
      Map<RigidBodyBasics, FootSwitchInterface> footSwitchMap = new HashMap<RigidBodyBasics, FootSwitchInterface>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactablePlaneBody = footContactableBodies.get().get(quadrant);
         RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();
         feetMap.put(rigidBody, contactablePlaneBody);
         FootSwitchInterface footSwitch = footSwitches.get().get(quadrant);
         footSwitchMap.put(rigidBody, footSwitch);
      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.get().getImuNames();
      double gravityMagnitude = Math.abs(gravity.get());

      StateEstimatorController stateEstimator = new DRCKinematicsBasedStateEstimator(inverseDynamicsStructure, stateEstimatorParameters.get(),
                                                                                     sensorOutputMapReadOnly.get(), centerOfMassDataHolder.get(),
                                                                                     imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitchMap,
                                                                                     centerOfPressureDataHolder, robotMotionStatusFromController.get(), feetMap, yoGraphicsListRegistry.get());

      FactoryTools.disposeFactory(this);

      return stateEstimator;
   }

   public void setSensorInformation(QuadrupedSensorInformation sensorInformation)
   {
      this.sensorInformation.set(sensorInformation);
   }

   public void setStateEstimatorParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.stateEstimatorParameters.set(stateEstimatorParameters);
   }

   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setSensorOutputMapReadOnly(SensorOutputMapReadOnly sensorOutputMapReadOnly)
   {
      this.sensorOutputMapReadOnly.set(sensorOutputMapReadOnly);
   }

   public void setFootContactableBodies(QuadrantDependentList<ContactablePlaneBody> footContactableBodies)
   {
      this.footContactableBodies.set(footContactableBodies);
   }

   public void setFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches)
   {
      this.footSwitches.set(footSwitches);
   }

   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setEstimatorDT(double estimatorDT)
   {
      this.estimatorDT.set(estimatorDT);
   }

   public void setYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistry.set(yoGraphicsListRegistry);
   }

   public void setCenterOfMassDataHolder(CenterOfMassDataHolder centerOfMassDataHolder)
   {
      this.centerOfMassDataHolder.set(centerOfMassDataHolder);
   }

   public void setRobotMotionStatusFromControllerHolder(RobotMotionStatusHolder robotMotionStatusFromControllerHolder)
   {
      this.robotMotionStatusFromController.set(robotMotionStatusFromControllerHolder);
   }
}
