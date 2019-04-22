package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
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

   private OptionalFactoryField<CenterOfMassDataHolder> estimatorCenterOfMassDataHolderToUpdateField = new OptionalFactoryField<>("estimatorCenterOfMassDataHolderToUpdateField");

   private RequiredFactoryField<CenterOfPressureDataHolder> centerOfPressureDataHolderFromControllerField = new RequiredFactoryField<>("centerOfPressureDataHolderFromControllerField");
   private RequiredFactoryField<RobotMotionStatusHolder> robotMotionStatusFromControllerField = new RequiredFactoryField<>("robotMotionStatusFromControllerField");

   private OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisPoseSubscriberField = new OptionalFactoryField<>("externalPelvisPoseSubscriberField");

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

   public KinematicsBasedStateEstimatorFactory setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      this.externalPelvisPoseSubscriberField.set(externalPelvisPoseSubscriber);
      return this;
   }

   public StateEstimatorController createStateEstimator(YoVariableRegistry stateEstimatorRegistry, YoGraphicsListRegistry stateEstimatorYoGraphicsListRegistry)
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

      Map<RigidBodyBasics, FootSwitchInterface> footSwitchMap = new LinkedHashMap<RigidBodyBasics, FootSwitchInterface>();
      Map<RigidBodyBasics, ContactablePlaneBody> bipedFeetMap = new LinkedHashMap<RigidBodyBasics, ContactablePlaneBody>();

      DRCRobotSensorInformation sensorInformation = sensorInformationField.get();
      ForceSensorDataHolder estimatorForceSensorDataHolderToUpdate = estimatorForceSensorDataHolderToUpdateField.get();
      StateEstimatorParameters stateEstimatorParameters = stateEstimatorParametersField.get();
      FootSwitchFactory footSwitchFactory = stateEstimatorParameters.getFootSwitchFactory();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footForceSensorName = sensorInformation.getFeetForceSensorNames().get(robotSide);
         ForceSensorDataReadOnly footForceSensorForEstimator = estimatorForceSensorDataHolderToUpdate.getByName(footForceSensorName);
         String namePrefix = bipedFeet.get(robotSide).getName() + "StateEstimator";

         RigidBodyBasics foot = bipedFeet.get(robotSide).getRigidBody();
         bipedFeetMap.put(foot, bipedFeet.get(robotSide));

         Set<ContactableFoot> otherFoot = Collections.singleton(bipedFeet.get(robotSide.getOppositeSide()));
         FootSwitchInterface footSwitch = footSwitchFactory.newFootSwitch(namePrefix, bipedFeet.get(robotSide), otherFoot, footForceSensorForEstimator,
                                                                          totalRobotWeight, null, stateEstimatorRegistry);
         footSwitchMap.put(foot, footSwitch);
      }

      String[] imuSensorsToUseInStateEstimator = sensorInformation.getIMUSensorsToUseInStateEstimator();

      // Create the sensor readers and state estimator here:
      CenterOfMassDataHolder estimatorCenterOfMassDataHolder = estimatorCenterOfMassDataHolderToUpdateField.hasValue()
            ? estimatorCenterOfMassDataHolderToUpdateField.get()
            : null;
      DRCKinematicsBasedStateEstimator estimator = new DRCKinematicsBasedStateEstimator(fullInverseDynamicsStructure, stateEstimatorParameters,
                                                                                        sensorOutputMapReadOnlyField.get(),
                                                                                        estimatorCenterOfMassDataHolder,
                                                                                        imuSensorsToUseInStateEstimator, gravityMagnitude, footSwitchMap,
                                                                                        centerOfPressureDataHolderFromControllerField.get(),
                                                                                        robotMotionStatusFromControllerField.get(), bipedFeetMap,
                                                                                        stateEstimatorYoGraphicsListRegistry);

      if (externalPelvisPoseSubscriberField.hasValue())
      {
         estimator.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriberField.get());
      }

      return estimator;
   }

   private FullInverseDynamicsStructure createFullInverseDynamicsStructure(FullHumanoidRobotModel fullRobotModel)
   {
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      FloatingJointBasics rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBodyBasics estimationLink = fullRobotModel.getPelvis();

      return new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);
   }
}
