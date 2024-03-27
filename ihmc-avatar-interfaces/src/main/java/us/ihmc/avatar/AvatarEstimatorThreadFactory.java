package us.ihmc.avatar;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.function.BooleanSupplier;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.stateEstimation.ekf.HumanoidRobotEKFWithSimpleJoints;
import us.ihmc.stateEstimation.ekf.LeggedRobotEKF;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorControllerFactory;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.lists.PairList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarEstimatorThreadFactory
{
   private final YoRegistry estimatorRegistry = new YoRegistry("DRCEstimatorThread");

   // Required fields -----------------------------------------------
   private final RequiredFactoryField<Double> gravityField = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<HumanoidRobotContextDataFactory> humanoidRobotContextDataFactoryField = new RequiredFactoryField<>("humanoidRobotContextDataFactory");
   private final RequiredFactoryField<FullHumanoidRobotModel> estimatorFullRobotModelField = new RequiredFactoryField<>("estimatorFullRobotModel");
   private final RequiredFactoryField<StateEstimatorParameters> stateEstimatorParametersField = new RequiredFactoryField<>("stateEstimatorParameters");
   private final RequiredFactoryField<SensorReaderFactory> sensorReaderFactoryField = new RequiredFactoryField<>("sensorReaderFactory");
   private final RequiredFactoryField<RobotContactPointParameters<RobotSide>> contactPointParametersField = new RequiredFactoryField<>("contactPointParameters");
   private final RequiredFactoryField<HumanoidRobotSensorInformation> sensorInformationField = new RequiredFactoryField<>("sensorInformation");
   private final RequiredFactoryField<WholeBodyControllerParameters<RobotSide>> controllerParametersField = new RequiredFactoryField<>("controllerParameters");

   // Optional fields -----------------------------------------------
   private final OptionalFactoryField<YoGraphicsListRegistry> yoGraphicsListRegistryField = new OptionalFactoryField<>("yoGraphicsListRegistry");
   private final OptionalFactoryField<StateEstimatorController> mainStateEstimatorField = new OptionalFactoryField<>("mainEstimatorController");
   private final OptionalFactoryField<PairList<BooleanSupplier, StateEstimatorController>> secondaryStateEstimatorsField = new OptionalFactoryField<>("secondaryEstimatorControllers");
   private final OptionalFactoryField<List<StateEstimatorControllerFactory>> secondaryStateEstimatorFactoriesField = new OptionalFactoryField<>("secondaryEstimatorControllerFactories");

   private final OptionalFactoryField<RobotConfigurationDataPublisher> robotConfigurationDataPublisherField = new OptionalFactoryField<>("robotConfigurationDataPublisher");

   private final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisPoseSubscriberField = new OptionalFactoryField<>("externalPelvisPoseSubscriberField");

   private final OptionalFactoryField<RealtimeROS2Node> realtimeROS2NodeField = new OptionalFactoryField<>("realtimeROS2Node");
   private final OptionalFactoryField<ROS2Topic<?>> outputTopicField = new OptionalFactoryField<>("outputTopic");
   private final OptionalFactoryField<ROS2Topic<?>> inputTopicField = new OptionalFactoryField<>("inputTopic");

   private final OptionalFactoryField<SensorDataContext> sensorDataContextField = new OptionalFactoryField<>("sensorDataContext");
   private final OptionalFactoryField<HumanoidRobotContextData> humanoidRobotContextDataField = new OptionalFactoryField<>("humanoidRobotContextData");
   private final OptionalFactoryField<HumanoidRobotContextJointData> humanoidRobotContextJointDataField = new OptionalFactoryField<>("humanoidRobotContextJointData");
   private final OptionalFactoryField<LowLevelOneDoFJointDesiredDataHolder> desiredJointDataHolderField = new OptionalFactoryField<>("desiredJointDataHolder");

   private final OptionalFactoryField<FloatingJointBasics> rootJointField = new OptionalFactoryField<>("rootJoint");
   private final OptionalFactoryField<OneDoFJointBasics[]> oneDoFJointsField = new OptionalFactoryField<>("oneDoFJoints");
   private final OptionalFactoryField<OneDoFJointBasics[]> controllableOneDoFJointsField = new OptionalFactoryField<>("controllableOneDoFJoints");

   private final OptionalFactoryField<RobotMotionStatusHolder> robotMotionStatusFromControllerField = new OptionalFactoryField<>("robotMotionStatusFromController");
   private final OptionalFactoryField<CenterOfPressureDataHolder> centerOfPressureDataHolderFromControllerField = new OptionalFactoryField<>("centerOfPressureDataHolderFromController");
   private final OptionalFactoryField<ForceSensorDataHolder> forceSensorDataHolderField = new OptionalFactoryField<>("forceSensorDataHolder");
   private final OptionalFactoryField<CenterOfMassDataHolder> centerOfMassDataHolderField = new OptionalFactoryField<>("centerOfMassDataHolder");
   private final OptionalFactoryField<ForceSensorDefinition[]> forceSensorDefinitionsField = new OptionalFactoryField<>("forceSensorDefinitionsField");
   private final OptionalFactoryField<IMUDefinition[]> imuDefinitionsField = new OptionalFactoryField<>("imuDefinitions");

   private final OptionalFactoryField<ContactableBodiesFactory<RobotSide>> contactableBodiesFactoryField = new OptionalFactoryField<>("contactableBodiesFactory");

   private final OptionalFactoryField<SensorReader> sensorReaderField = new OptionalFactoryField<>("sensorReader");
   private final OptionalFactoryField<SensorOutputMapReadOnly> rawSensorOutputMapField = new OptionalFactoryField<>("rawSensorOutputMap");
   private final OptionalFactoryField<SensorOutputMapReadOnly> processedSensorOutputMapField = new OptionalFactoryField<>("processedSensorOutputMap");

   private final OptionalFactoryField<JointDesiredOutputWriter> jointDesiredOutputWriterField = new OptionalFactoryField<>("jointDesiredOutputWriter");

   /**
    * Creates a new factory to create {@link AvatarEstimatorThread}.
    * <p>
    * Example for configuring this factory:
    *
    * <pre>
    * AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
    * avatarEstimatorThreadFactory.setROS2Info(realtimeROS2Node, robotName);
    * avatarEstimatorThreadFactory.configureWithDRCRobotModel(robotModel);
    * avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
    * avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(contextDataFactory);
    * avatarEstimatorThreadFactory.setExternalPelvisCorrectorSubscriber(pelvisPoseCorrectionCommunicator);
    * avatarEstimatorThreadFactory.setJointDesiredOutputWriter(simulationOutputWriter);
    * avatarEstimatorThreadFactory.setGravity(gravity);
    * AvatarEstimatorThread estimatorThread = avatarEstimatorThreadFactory.createAvatarEstimatorThread();
    * </pre>
    *
    * Where {@code robotModel} is an implementation of {@link DRCRobotModel}.
    * </p>
    */
   public AvatarEstimatorThreadFactory()
   {
   }

   /**
    * Sets the magnitude of gravity to use in the state estimator, the sign of the value is not
    * considered.
    *
    * @param gravity magnitude of the gravitational acceleration.
    */
   public void setGravity(double gravity)
   {
      gravityField.set(gravity);
   }

   /**
    * Configure this factory as follows:
    * <ul>
    * <li>Set the full-robot model using {@link DRCRobotModel#createFullRobotModel()}.
    * <li>Set the controller parameters using {@link DRCRobotModel}.
    * <li>Set the state estimator parameters using {@link DRCRobotModel#getStateEstimatorParameters()}.
    * <li>Set the sensor information using {@link DRCRobotModel#getSensorInformation()}.
    * <li>Set the contact point parameters using {@link DRCRobotModel#getContactPointParameters()}.
    * </ul>
    *
    * @param robotModel        the robot model used to configure this factory.
    * @param robotInitialSetup
    */
   public void configureWithDRCRobotModel(DRCRobotModel robotModel)
   {
      configureWithDRCRobotModel(robotModel, null);
   }

   public void configureWithDRCRobotModel(DRCRobotModel robotModel, RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      configureWithWholeBodyControllerParameters(robotModel);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      if (robotInitialSetup != null)
      {
         robotInitialSetup.initializeFullRobotModel(fullRobotModel);
      }
      setEstimatorFullRobotModel(fullRobotModel);
   }

   /**
    * Configure this factory as follows:
    * <ul>
    * <li>Set the controller parameters using {@link WholeBodyControllerParameters}.
    * <li>Set the state estimator parameters using
    * {@link WholeBodyControllerParameters#getStateEstimatorParameters()}.
    * <li>Set the sensor information using
    * {@link WholeBodyControllerParameters#getSensorInformation()}.
    * <li>Set the contact point parameters using
    * {@link WholeBodyControllerParameters#getContactPointParameters()}.
    * </ul>
    *
    * @param wholeBodyControllerParameters the parameters used to configure this factory.
    */
   public void configureWithWholeBodyControllerParameters(WholeBodyControllerParameters<RobotSide> wholeBodyControllerParameters)
   {
      setConrollerParameters(wholeBodyControllerParameters);
      setStateEstimatorParamters(wholeBodyControllerParameters.getStateEstimatorParameters());
      setSensorInformation(wholeBodyControllerParameters.getSensorInformation());
      setContactPointParameters(wholeBodyControllerParameters.getContactPointParameters());
   }

   /**
    * ROS 2 necessary information to create the real-time publisher/subscriber.
    *
    * @param ros2Node  the real-time node to create the publisher with.
    * @param robotName the name of the robot used to get the topic name generator, see
    *                  {@link ControllerAPIDefinition#getOutputTopic(String)} and
    *                  {@link ControllerAPIDefinition#getInputTopic(String)}.
    */
   public void setROS2Info(RealtimeROS2Node ros2Node, String robotName)
   {
      setROS2Info(ros2Node, ROS2Tools.getControllerOutputTopic(robotName), ROS2Tools.getControllerInputTopic(robotName));
   }

   /**
    * ROS 2 necessary information to create the real-time publisher/subscriber.
    *
    * @param ros2Node    the real-time node to create the publisher with.
    * @param outputTopic the generator to use for creating the topic name for publishers.
    * @param inputTopic  the generator to use for creating the topic name for subscribers.
    */
   public void setROS2Info(RealtimeROS2Node ros2Node, ROS2Topic<?> outputTopic, ROS2Topic<?> inputTopic)
   {
      realtimeROS2NodeField.set(ros2Node);
      outputTopicField.set(outputTopic);
      inputTopicField.set(inputTopic);
   }

   /**
    * Sets the full-robot model that is to be used by the state estimator.
    *
    * @param estimatorFullRobotModel the full-robot model
    */
   public void setEstimatorFullRobotModel(FullHumanoidRobotModel estimatorFullRobotModel)
   {
      estimatorFullRobotModelField.set(estimatorFullRobotModel);
   }

   /**
    * Sets the sensor information that is to be used by the state estimator to retrieve sensors such as
    * wrist force/torque sensors.
    *
    * @param sensorInformation the sensor information.
    */
   public void setSensorInformation(HumanoidRobotSensorInformation sensorInformation)
   {
      sensorInformationField.set(sensorInformation);
   }

   /**
    * Sets the contact point parameters that is used by the state estimator for creating contactable
    * bodies.
    *
    * @param contactPointParameters the contact point parameters.
    */
   public void setContactPointParameters(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      contactPointParametersField.set(contactPointParameters);
   }

   /**
    * Sets the state estimator parameters used to configure things such as filters.
    *
    * @param stateEstimatorParameters the state estimator parameters.
    */
   public void setStateEstimatorParamters(StateEstimatorParameters stateEstimatorParameters)
   {
      stateEstimatorParametersField.set(stateEstimatorParameters);
   }

   /**
    * Sets the parameters required to load Yo parameters, see
    * {@link ParameterLoaderHelper#loadParameters(Object, WholeBodyControllerParameters, YoRegistry)}.
    *
    * @param controllerParameters the controller parameters.
    */
   public void setConrollerParameters(WholeBodyControllerParameters<RobotSide> controllerParameters)
   {
      controllerParametersField.set(controllerParameters);
   }

   /**
    * Sets the factory used to create the sensor reader.
    *
    * @param sensorReaderFactory the sensor reader factory.
    */
   public void setSensorReaderFactory(SensorReaderFactory sensorReaderFactory)
   {
      sensorReaderFactoryField.set(sensorReaderFactory);
   }

   /**
    * The factory to create the context for the state estimator needed to run with the
    * {@link BarrierScheduler}.
    *
    * @param contextDataFactory the context factory.
    */
   public void setHumanoidRobotContextDataFactory(HumanoidRobotContextDataFactory contextDataFactory)
   {
      humanoidRobotContextDataFactoryField.set(contextDataFactory);
   }

   /**
    * Optional: sets the subscriber to receive pelvis pose update from an external module that performs
    * localization.
    * <p>
    * This is used internally to the state estimator to slowly correct the drift on the estimated robot
    * position.
    * </p>
    *
    * @param externalPelvisPoseSubscriber the pelvis pose subscriber.
    */
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      externalPelvisPoseSubscriberField.set(externalPelvisPoseSubscriber);
   }

   /**
    * Optional: sets the low-level output writer, this is rarely needed.
    *
    * @param jointDesiredOutputWriter the joint desired output writer.
    */
   public void setJointDesiredOutputWriter(JointDesiredOutputWriter jointDesiredOutputWriter)
   {
      if (jointDesiredOutputWriter != null)
         jointDesiredOutputWriterField.set(jointDesiredOutputWriter);
   }

   /**
    * Optional: sets a custom publisher for {@link RobotConfigurationData}.
    *
    * @param publisher the custom publisher.
    */
   public void setRobotConfigurationDataPublisher(RobotConfigurationDataPublisher publisher)
   {
      if (publisher != null)
         robotConfigurationDataPublisherField.set(publisher);
   }

   /**
    * Optional: sets the main state estimator to use.
    * <p>
    * Two distinct state estimators can be built with this factory using:
    * <ul>
    * <li>{@link #createDRCKinematicsStateEstimator()} to create the default main state estimator.
    * <li>{@link #createEKFStateEstimator()} to create a state estimator that is based on an Extended
    * Kalman Filter.
    * </ul>
    * </p>
    *
    * @param mainStateEstimator the instance of the main state estimator.
    */
   public void setMainStateEstimator(StateEstimatorController mainStateEstimator)
   {
      if (mainStateEstimatorField.hasValue())
         throw new IllegalOperationException("The main state estimator has already been set.");
      mainStateEstimatorField.set(mainStateEstimator);
   }

   /**
    * Optional: adds a secondary state estimator to run in parallel to the main state estimator.
    *
    * @param secondaryStateEstimator the secondary state estimator.
    */
   public void addSecondaryStateEstimator(StateEstimatorController secondaryStateEstimator)
   {
      addSecondaryStateEstimators(() -> false, secondaryStateEstimator);
   }


   /**
    * Optional: adds a secondary state estimator factory to run in parallel to the main state estimator.
    *
    * @param secondaryStateEstimator the secondary state estimator.
    */
   public void addSecondaryStateEstimatorFactory(StateEstimatorControllerFactory secondaryStateEstimator)
   {
      getSecondaryStateEstimatorFactories().add(secondaryStateEstimator);
   }

   /**
    * Optional: adds a secondary state estimator to run in parallel to the main state estimator.
    *
    * @param reinitializeVariableName name of the {@link YoBoolean} to create that can be used via SCS
    *                                 to manually reinitilize the secondary state estimator.
    * @param secondaryStateEstimator  the secondary state estimator.
    */
   public void addSecondaryStateEstimator(String reinitializeVariableName, StateEstimatorController secondaryStateEstimator)
   {
      YoBoolean reinitialize = new YoBoolean(reinitializeVariableName, getEstimatorRegistry());
      BooleanSupplier reinitilizeSupplier = () ->
      {
         boolean ret = reinitialize.getValue();
         reinitialize.set(false);
         return ret;
      };
      addSecondaryStateEstimators(reinitilizeSupplier, secondaryStateEstimator);
   }

   /**
    * Optional: adds a secondary state estimator to run in parallel to the main state estimator.
    *
    * @param reinitilizeSupplier     function check at every tick to determine whether the secondary
    *                                state estimator should be reinitilized. See
    *                                {@link AvatarEstimatorThread#run()}.
    * @param secondaryStateEstimator the secondary state estimator.
    */
   public void addSecondaryStateEstimators(BooleanSupplier reinitilizeSupplier, StateEstimatorController secondaryStateEstimator)
   {
      if (!useStateEstimator())
         throw new IllegalOperationException("Cannot add state estimator because SensorReaderFactory.useStateEstimator() is false.");
      getSecondaryStateEstimators().add(reinitilizeSupplier, secondaryStateEstimator);
   }

   public void setYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoGraphicsListRegistryField.set(yoGraphicsListRegistry);
   }

   public AvatarEstimatorThread createAvatarEstimatorThread()
   {
      if (jointDesiredOutputWriterField.hasValue())
      {
         jointDesiredOutputWriterField.get().setJointDesiredOutputList(getDesiredJointDataHolder());
         getEstimatorRegistry().addChild(jointDesiredOutputWriterField.get().getYoVariableRegistry());
      }
      if (secondaryStateEstimatorFactoriesField.hasValue())
      {
         for (StateEstimatorControllerFactory stateEstimatorControllerFactory : secondaryStateEstimatorFactoriesField.get())
            addSecondaryStateEstimator(stateEstimatorControllerFactory.createStateEstimator(getEstimatorFullRobotModel(), getSensorReader()));
      }

      AvatarEstimatorThread avatarEstimatorThread = new AvatarEstimatorThread(getSensorReader(),
                                                                              getEstimatorFullRobotModel(),
                                                                              getHumanoidRobotContextData(),
                                                                              getMainStateEstimator(),
                                                                              getSecondaryStateEstimators(),
                                                                              createControllerCrashPublisher(),
                                                                              getEstimatorRegistry(),
                                                                              getYoGraphicsListRegistry());

      avatarEstimatorThread.addRobotController(new RobotJointLimitWatcher(getEstimatorFullRobotModel().getOneDoFJoints(), getRawSensorOutputMap()));
      RobotConfigurationDataPublisher robotConfigurationDataPublisher = getRobotConfigurationDataPublisher();
      if (robotConfigurationDataPublisher != null)
      {
         avatarEstimatorThread.setRawOutputWriter(robotConfigurationDataPublisher);
      }
      ParameterLoaderHelper.loadParameters(this, getControllerParameters(), getEstimatorRegistry());

      FactoryTools.disposeFactory(this);
      return avatarEstimatorThread;
   }

   public StateEstimatorController createDRCKinematicsStateEstimator()
   {
      if (!useStateEstimator())
         return null;

      // Create DRC Estimator:
      KinematicsBasedStateEstimatorFactory estimatorFactory = new KinematicsBasedStateEstimatorFactory();
      estimatorFactory.setEstimatorFullRobotModel(getEstimatorFullRobotModel());
      estimatorFactory.setSensorInformation(getSensorInformation());
      estimatorFactory.setSensorOutputMapReadOnly(getProcessedSensorOutputMap());
      estimatorFactory.setGravity(getGravity());
      estimatorFactory.setStateEstimatorParameters(getStateEstimatorParameters());
      estimatorFactory.setContactableBodiesFactory(getContactableBodiesFactory());
      estimatorFactory.setEstimatorForceSensorDataHolder(getForceSensorDataHolder());
      estimatorFactory.setEstimatorCenterOfMassDataHolderToUpdate(getCenterOfMassDataHolder());
      estimatorFactory.setCenterOfPressureDataHolderFromController(getCenterOfPressureDataHolderFromController());
      estimatorFactory.setRobotMotionStatusFromController(getRobotMotionStatusFromController());
      estimatorFactory.setExternalPelvisCorrectorSubscriber(getExternalPelvisPoseSubscriberField());
      DRCKinematicsBasedStateEstimator stateEstimator = estimatorFactory.createStateEstimator(getEstimatorRegistry(), getYoGraphicsListRegistry());

      if (realtimeROS2NodeField.hasValue())
      {
         ForceSensorStateUpdater forceSensorStateUpdater = stateEstimator.getForceSensorStateUpdater();
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2NodeField.get(),
                                                       RequestWristForceSensorCalibrationPacket.class,
                                                       inputTopicField.get(),
                                                       subscriber -> forceSensorStateUpdater.requestWristForceSensorCalibrationAtomic());
      }

      return stateEstimator;
   }

   public StateEstimatorController createEKFStateEstimator()
   {
      if (!useStateEstimator())
         return null;

      double estimatorDT = getStateEstimatorParameters().getEstimatorDT();
      HumanoidRobotSensorInformation sensorInformation = getSensorInformation();
      SideDependentList<String> footForceSensorNames = sensorInformation.getFeetForceSensorNames();
      String primaryImuName = sensorInformation.getPrimaryBodyImu();
      Collection<String> imuSensorNames = Arrays.asList(sensorInformation.getIMUSensorsToUseInStateEstimator());
      HumanoidRobotEKFWithSimpleJoints ekfStateEstimator = new HumanoidRobotEKFWithSimpleJoints(getEstimatorFullRobotModel(),
                                                                                                primaryImuName,
                                                                                                imuSensorNames,
                                                                                                footForceSensorNames,
                                                                                                getRawSensorOutputMap(),
                                                                                                estimatorDT,
                                                                                                getGravity(),
                                                                                                getProcessedSensorOutputMap(),
                                                                                                getYoGraphicsListRegistry(),
                                                                                                getEstimatorFullRobotModel());

      InputStream ekfParameterStream = LeggedRobotEKF.class.getResourceAsStream("/ekf.xml");
      if (ekfParameterStream == null)
      {
         throw new RuntimeException("Did not find parameter file for EKF.");
      }
      ParameterLoaderHelper.loadParameters(this, ekfParameterStream, ekfStateEstimator.getYoRegistry());
      return ekfStateEstimator;
   }

   public RealtimeROS2Node getRealtimeROS2Node()
   {
      if (realtimeROS2NodeField.hasValue())
         return realtimeROS2NodeField.get();
      else
         return null;
   }

   public ROS2Topic<?> getOutputTopic()
   {
      if (outputTopicField.hasValue())
         return outputTopicField.get();
      else
         return null;
   }

   public ROS2Topic<?> getInputTopic()
   {
      if (inputTopicField.hasValue())
         return inputTopicField.get();
      else
         return null;
   }

   private ROS2PublisherBasics<ControllerCrashNotificationPacket> createControllerCrashPublisher()
   {
      if (realtimeROS2NodeField.hasValue())
         return ROS2Tools.createPublisherTypeNamed(realtimeROS2NodeField.get(), ControllerCrashNotificationPacket.class, outputTopicField.get());
      else
         return null;
   }

   public Double getGravity()
   {
      return gravityField.get();
   }

   public HumanoidRobotContextDataFactory getHumanoidRobotContextDataFactory()
   {
      return humanoidRobotContextDataFactoryField.get();
   }

   public HumanoidRobotContextJointData getHumanoidRobotContextJointData()
   {
      if (!humanoidRobotContextJointDataField.hasValue())
         humanoidRobotContextJointDataField.set(new HumanoidRobotContextJointData(getOneDoFJoints().length));
      return humanoidRobotContextJointDataField.get();
   }

   public SensorDataContext getSensorDataContext()
   {
      if (!sensorDataContextField.hasValue())
         sensorDataContextField.set(new SensorDataContext(getEstimatorFullRobotModel()));
      return sensorDataContextField.get();
   }

   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      if (!humanoidRobotContextDataField.hasValue())
      {
         HumanoidRobotContextDataFactory contextDataFactory = getHumanoidRobotContextDataFactory();
         contextDataFactory.setForceSensorDataHolder(getForceSensorDataHolder());
         contextDataFactory.setCenterOfMassDataHolder(getCenterOfMassDataHolder());
         contextDataFactory.setCenterOfPressureDataHolder(getCenterOfPressureDataHolderFromController());
         contextDataFactory.setRobotMotionStatusHolder(getRobotMotionStatusFromController());
         contextDataFactory.setJointDesiredOutputList(getDesiredJointDataHolder());
         contextDataFactory.setProcessedJointData(getHumanoidRobotContextJointData());
         contextDataFactory.setSensorDataContext(getSensorDataContext());
         humanoidRobotContextDataField.set(contextDataFactory.createHumanoidRobotContextData());
      }
      return humanoidRobotContextDataField.get();
   }

   public LowLevelOneDoFJointDesiredDataHolder getDesiredJointDataHolder()
   {
      if (!desiredJointDataHolderField.hasValue())
         desiredJointDataHolderField.set(new LowLevelOneDoFJointDesiredDataHolder(getControllableOneDoFJoints()));
      return desiredJointDataHolderField.get();
   }

   public RobotMotionStatusHolder getRobotMotionStatusFromController()
   {
      if (!robotMotionStatusFromControllerField.hasValue())
         robotMotionStatusFromControllerField.set(new RobotMotionStatusHolder());
      return robotMotionStatusFromControllerField.get();
   }

   public CenterOfPressureDataHolder getCenterOfPressureDataHolderFromController()
   {
      if (!centerOfPressureDataHolderFromControllerField.hasValue())
         centerOfPressureDataHolderFromControllerField.set(new CenterOfPressureDataHolder(getEstimatorFullRobotModel()));
      return centerOfPressureDataHolderFromControllerField.get();
   }

   public ForceSensorDataHolder getForceSensorDataHolder()
   {
      if (!forceSensorDataHolderField.hasValue())
         forceSensorDataHolderField.set(new ForceSensorDataHolder(getForceSensorDefinitions()));
      return forceSensorDataHolderField.get();
   }

   public CenterOfMassDataHolder getCenterOfMassDataHolder()
   {
      if (!centerOfMassDataHolderField.hasValue())
         centerOfMassDataHolderField.set(new CenterOfMassDataHolder());
      return centerOfMassDataHolderField.get();
   }

   public ForceSensorDefinition[] getForceSensorDefinitions()
   {
      if (!forceSensorDefinitionsField.hasValue())
         forceSensorDefinitionsField.set(getEstimatorFullRobotModel().getForceSensorDefinitions());
      return forceSensorDefinitionsField.get();
   }

   public IMUDefinition[] getIMUDefinitions()
   {
      if (!imuDefinitionsField.hasValue())
         imuDefinitionsField.set(getEstimatorFullRobotModel().getIMUDefinitions());
      return imuDefinitionsField.get();
   }

   public SensorReaderFactory getSensorReaderFactory()
   {
      return sensorReaderFactoryField.get();
   }

   public boolean useStateEstimator()
   {
      return getSensorReaderFactory().useStateEstimator();
   }

   public SensorReader getSensorReader()
   {
      if (!sensorReaderField.hasValue())
      {
         // This is only used by the perfect sensor reader if we are not using the estimator. It will update the data structure.
         getSensorReaderFactory().setForceSensorDataHolder(getForceSensorDataHolder());
         getSensorReaderFactory().build(getRootJoint(), getIMUDefinitions(), getForceSensorDefinitions(), getDesiredJointDataHolder(), getEstimatorRegistry());
         sensorReaderField.set(getSensorReaderFactory().getSensorReader());
      }
      return sensorReaderField.get();
   }

   public SensorOutputMapReadOnly getRawSensorOutputMap()
   {
      if (!rawSensorOutputMapField.hasValue())
         rawSensorOutputMapField.set(getSensorReader().getRawSensorOutputMap());
      return rawSensorOutputMapField.get();
   }

   public SensorOutputMapReadOnly getProcessedSensorOutputMap()
   {
      if (!processedSensorOutputMapField.hasValue())
         processedSensorOutputMapField.set(getSensorReader().getProcessedSensorOutputMap());
      return processedSensorOutputMapField.get();
   }

   public FullHumanoidRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModelField.get();
   }

   public FloatingJointBasics getRootJoint()
   {
      if (!rootJointField.hasValue())
         rootJointField.set(getEstimatorFullRobotModel().getRootJoint());
      return rootJointField.get();
   }

   public OneDoFJointBasics[] getOneDoFJoints()
   {
      if (!oneDoFJointsField.hasValue())
         oneDoFJointsField.set(getEstimatorFullRobotModel().getOneDoFJoints());
      return oneDoFJointsField.get();
   }

   public OneDoFJointBasics[] getControllableOneDoFJoints()
   {
      if (!controllableOneDoFJointsField.hasValue())
         controllableOneDoFJointsField.set(getEstimatorFullRobotModel().getControllableOneDoFJoints());
      return controllableOneDoFJointsField.get();
   }

   public ContactableBodiesFactory<RobotSide> getContactableBodiesFactory()
   {
      if (!contactableBodiesFactoryField.hasValue())
      {
         RobotContactPointParameters<RobotSide> contactPointParameters = getContactPointParameters();
         List<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
         List<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
         List<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();
         SegmentDependentList<RobotSide, ArrayList<Point2D>> footContactPoints = contactPointParameters.getFootContactPoints();
         SegmentDependentList<RobotSide, Point2D> toeContactPoints = contactPointParameters.getControllerToeContactPoints();
         SegmentDependentList<RobotSide, LineSegment2D> toeContactLines = contactPointParameters.getControllerToeContactLines();

         ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
         contactableBodiesFactory.setFootContactPoints(footContactPoints);
         contactableBodiesFactory.setToeContactParameters(toeContactPoints, toeContactLines);
         for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         {
            contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                               additionalContactNames.get(i),
                                                               additionalContactTransforms.get(i));
         }
         contactableBodiesFactoryField.set(contactableBodiesFactory);
      }

      return contactableBodiesFactoryField.get();
   }

   public RobotContactPointParameters<RobotSide> getContactPointParameters()
   {
      return contactPointParametersField.get();
   }

   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParametersField.get();
   }

   public WholeBodyControllerParameters<RobotSide> getControllerParameters()
   {
      return controllerParametersField.get();
   }

   public HumanoidRobotSensorInformation getSensorInformation()
   {
      return sensorInformationField.get();
   }

   public PelvisPoseCorrectionCommunicatorInterface getExternalPelvisPoseSubscriberField()
   {
      if (externalPelvisPoseSubscriberField.hasValue())
         return externalPelvisPoseSubscriberField.get();
      else
         return null;
   }

   public StateEstimatorController getMainStateEstimator()
   {
      if (!mainStateEstimatorField.hasValue())
         mainStateEstimatorField.set(createDRCKinematicsStateEstimator());
      return mainStateEstimatorField.get();
   }

   public PairList<BooleanSupplier, StateEstimatorController> getSecondaryStateEstimators()
   {
      if (!secondaryStateEstimatorsField.hasValue())
         secondaryStateEstimatorsField.set(new PairList<>());
      return secondaryStateEstimatorsField.get();
   }

   public List<StateEstimatorControllerFactory> getSecondaryStateEstimatorFactories()
   {
      if (!secondaryStateEstimatorFactoriesField.hasValue())
         secondaryStateEstimatorFactoriesField.set(new ArrayList<>());
      return secondaryStateEstimatorFactoriesField.get();
   }

   public RobotConfigurationDataPublisher getRobotConfigurationDataPublisher()
   {
      if (!realtimeROS2NodeField.hasValue())
         return null;

      if (!robotConfigurationDataPublisherField.hasValue())
      {
         ForceSensorDataHolderReadOnly forceSensorDataHolderToSend = getForceSensorDataHolder();
         if (getMainStateEstimator() != null && getMainStateEstimator().getForceSensorOutputWithGravityCancelled() != null)
            forceSensorDataHolderToSend = getMainStateEstimator().getForceSensorOutputWithGravityCancelled();

         RobotConfigurationDataPublisherFactory factory = new RobotConfigurationDataPublisherFactory();
         factory.setDefinitionsToPublish(getEstimatorFullRobotModel());
         factory.setSensorSource(getEstimatorFullRobotModel(), forceSensorDataHolderToSend, getRawSensorOutputMap());
         factory.setRobotMotionStatusHolder(getRobotMotionStatusFromController());
         factory.setROS2Info(realtimeROS2NodeField.get(), outputTopicField.get());
         factory.setPublishPeriod(Conversions.secondsToNanoseconds(StateEstimatorParameters.ROBOT_CONFIGURATION_DATA_PUBLISH_DT));
         robotConfigurationDataPublisherField.set(factory.createRobotConfigurationDataPublisher());
      }
      return robotConfigurationDataPublisherField.get();
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      if (!yoGraphicsListRegistryField.hasValue())
         yoGraphicsListRegistryField.set(new YoGraphicsListRegistry());
      return yoGraphicsListRegistryField.get();
   }

   public YoRegistry getEstimatorRegistry()
   {
      return estimatorRegistry;
   }
}
