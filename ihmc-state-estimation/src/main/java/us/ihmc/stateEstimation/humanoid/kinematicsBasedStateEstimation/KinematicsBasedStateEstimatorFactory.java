package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchAndContactSensorFusedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ContactSensor;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class KinematicsBasedStateEstimatorFactory
{
   private RequiredFactoryField<FullHumanoidRobotModel> estimatorFullRobotModelField = new RequiredFactoryField<>("estimatorFullRobotModelField");
   private RequiredFactoryField<DRCRobotSensorInformation> sensorInformationField = new RequiredFactoryField<>("sensorInformationField");

   private RequiredFactoryField<SensorOutputMapReadOnly> sensorOutputMapReadOnlyField = new RequiredFactoryField<>("sensorOutputMapReadOnlyField");
   private RequiredFactoryField<Double> gravityField = new RequiredFactoryField<>("gravityField");
   private RequiredFactoryField<StateEstimatorParameters> stateEstimatorParametersField = new RequiredFactoryField<>("stateEstimatorParametersField");

   private RequiredFactoryField<ContactableBodiesFactory<RobotSide>> contactableBodiesFactoryField = new RequiredFactoryField<>("contactableBodiesFactoryField");
   private RequiredFactoryField<ForceSensorDataHolder> estimatorForceSensorDataHolderToUpdateField = new RequiredFactoryField<>("estimatorForceSensorDataHolderToUpdateField");

   private RequiredFactoryField<CenterOfMassDataHolder> estimatorCenterOfMassDataHolderToUpdateField = new RequiredFactoryField<>("estimatorCenterOfMassDataHolderToUpdateField");

   private RequiredFactoryField<ContactSensorHolder> contactSensorHolderField = new RequiredFactoryField<>("contactSensorHolderField");

   private RequiredFactoryField<CenterOfPressureDataHolder> centerOfPressureDataHolderFromControllerField = new RequiredFactoryField<>("centerOfPressureDataHolderFromControllerField");
   private RequiredFactoryField<RobotMotionStatusHolder> robotMotionStatusFromControllerField = new RequiredFactoryField<>("robotMotionStatusFromControllerField");

   public KinematicsBasedStateEstimatorFactory setEstimatorFullRobotModel(FullHumanoidRobotModel estimatorFullRobotModel)
   {
      this.estimatorFullRobotModelField.set(estimatorFullRobotModel);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setSensorInformation(DRCRobotSensorInformation sensorInformation)
   {
      this.sensorInformationField.set(sensorInformation);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setSensorOutputMapReadOnly(SensorOutputMapReadOnly sensorOutputMapReadOnly)
   {
      this.sensorOutputMapReadOnlyField.set(sensorOutputMapReadOnly);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setGravity(Double gravity)
   {
      this.gravityField.set(gravity);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setStateEstimatorParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.stateEstimatorParametersField.set(stateEstimatorParameters);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setContactableBodiesFactory(ContactableBodiesFactory contactableBodiesFactory)
   {
      this.contactableBodiesFactoryField.set(contactableBodiesFactory);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setEstimatorForceSensorDataHolderToUpdate(ForceSensorDataHolder estimatorForceSensorDataHolderToUpdate)
   {
      this.estimatorForceSensorDataHolderToUpdateField.set(estimatorForceSensorDataHolderToUpdate);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setEstimatorCenterOfMassDataHolderToUpdate(CenterOfMassDataHolder estimatorCenterOfMassDataHolderToUpdate)
   {
      this.estimatorCenterOfMassDataHolderToUpdateField.set(estimatorCenterOfMassDataHolderToUpdate);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setContactSensorHolder(ContactSensorHolder contactSensorHolder)
   {
      this.contactSensorHolderField.set(contactSensorHolder);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setCenterOfPressureDataHolderFromController(CenterOfPressureDataHolder centerOfPressureDataHolderFromController)
   {
      this.centerOfPressureDataHolderFromControllerField.set(centerOfPressureDataHolderFromController);
      return this;
   }

   public KinematicsBasedStateEstimatorFactory setRobotMotionStatusFromController(RobotMotionStatusHolder robotMotionStatusFromController)
   {
      this.robotMotionStatusFromControllerField.set(robotMotionStatusFromController);
      return this;
   }

   public DRCKinematicsBasedStateEstimator createStateEstimator(YoVariableRegistry stateEstimatorRegistry,
                                                                YoGraphicsListRegistry stateEstimatorYoGraphicsListRegistry)
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      FullHumanoidRobotModel fullRobotModel = estimatorFullRobotModelField.get();

      FullInverseDynamicsStructure fullInverseDynamicsStructure = createFullInverseDynamicsStructure(fullRobotModel);

      HumanoidReferenceFrames estimatorReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = contactableBodiesFactoryField.get();
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(estimatorReferenceFrames);
      SideDependentList<ContactableFoot> bipedFeet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());

      double gravity = gravityField.get();
      double gravityMagnitude = Math.abs(gravity);
      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityMagnitude;

      Map<RigidBody, FootSwitchInterface> footSwitchMap = new LinkedHashMap<RigidBody, FootSwitchInterface>();
      Map<RigidBody, ContactablePlaneBody> bipedFeetMap = new LinkedHashMap<RigidBody, ContactablePlaneBody>();

      DRCRobotSensorInformation sensorInformation = sensorInformationField.get();
      ForceSensorDataHolder estimatorForceSensorDataHolderToUpdate = estimatorForceSensorDataHolderToUpdateField.get();
      StateEstimatorParameters stateEstimatorParameters = stateEstimatorParametersField.get();
      ContactSensorHolder contactSensorHolder = contactSensorHolderField.get();

      DoubleProvider contactThresholdForce = new DoubleParameter("ContactThresholdForce", stateEstimatorRegistry, stateEstimatorParameters.getContactThresholdForce());
      DoubleProvider copThresholdFraction = new DoubleParameter("CoPThresholdFraction", stateEstimatorRegistry, stateEstimatorParameters.getFootSwitchCoPThresholdFraction());
      DoubleProvider contactThresholdHeight = new DoubleParameter("ContactThresholdHeight", stateEstimatorRegistry, stateEstimatorParameters.getContactThresholdHeight());

      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         String footContactSensorName = sensorInformation.getFeetContactSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = estimatorForceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         RigidBody foot = bipedFeet.get(robotSide).getRigidBody();
         bipedFeetMap.put(foot, bipedFeet.get(robotSide));

         switch (stateEstimatorParameters.getFootSwitchType())
         {
         case KinematicBased:

            KinematicsBasedFootSwitch footSwitch = new KinematicsBasedFootSwitch(namePrefix, bipedFeet, contactThresholdHeight, totalRobotWeight, robotSide,
                                                                                 stateEstimatorRegistry);
            footSwitchMap.put(foot, footSwitch);
            break;
         case WrenchBased:
            WrenchBasedFootSwitch wrenchBasedFootSwitchForEstimator = new WrenchBasedFootSwitch(namePrefix, footForceSensorForEstimator, totalRobotWeight,
                                                                                                bipedFeet.get(robotSide), contactThresholdForce, null,
                                                                                                copThresholdFraction, null, stateEstimatorRegistry);
            footSwitchMap.put(foot, wrenchBasedFootSwitchForEstimator);
            break;

         case WrenchAndContactSensorFused:
            ContactSensor footContactSensor = contactSensorHolder.getByName(footContactSensorName);
            WrenchAndContactSensorFusedFootSwitch wrenchAndContactSensorBasedFootswitch = new WrenchAndContactSensorFusedFootSwitch(namePrefix,
                                                                                                                                    footForceSensorForEstimator,
                                                                                                                                    footContactSensor,
                                                                                                                                    totalRobotWeight,
                                                                                                                                    bipedFeet.get(robotSide),
                                                                                                                                    contactThresholdForce, null,
                                                                                                                                    copThresholdFraction, null,
                                                                                                                                    stateEstimatorRegistry);
            footSwitchMap.put(foot, wrenchAndContactSensorBasedFootswitch);
            break;
         default:
            throw new Error("unknown foot switch type");
         }

      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();

      // Create the sensor readers and state estimator here:

      return new DRCKinematicsBasedStateEstimator(fullInverseDynamicsStructure, stateEstimatorParameters, sensorOutputMapReadOnlyField.get(),
                                                  estimatorForceSensorDataHolderToUpdate, estimatorCenterOfMassDataHolderToUpdateField.get(),
                                                  imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitchMap,
                                                  centerOfPressureDataHolderFromControllerField.get(), robotMotionStatusFromControllerField.get(), bipedFeetMap,
                                                  stateEstimatorYoGraphicsListRegistry);
   }

   private FullInverseDynamicsStructure createFullInverseDynamicsStructure(FullHumanoidRobotModel fullRobotModel)
   {
      RigidBody elevator = fullRobotModel.getElevator();
      FloatingInverseDynamicsJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBody estimationLink = fullRobotModel.getPelvis();

      return new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);
   }
}
