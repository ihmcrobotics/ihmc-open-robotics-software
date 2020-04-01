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
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.stateEstimation.ekf.HumanoidRobotEKFWithSimpleJoints;
import us.ihmc.stateEstimation.ekf.LeggedRobotEKF;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.lists.PairList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarEstimatorThreadFactory
{
   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

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
   private final OptionalFactoryField<StateEstimatorController> mainStateEstimatorField = new OptionalFactoryField<>("mainEstimatorController");
   private final OptionalFactoryField<PairList<BooleanSupplier, StateEstimatorController>> secondaryStateEstimatorsField = new OptionalFactoryField<>("secondaryEstimatorControllers");

   private final OptionalFactoryField<RobotConfigurationDataPublisher> robotConfigurationDataPublisherField = new OptionalFactoryField<>("robotConfigurationDataPublisher");

   private final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisPoseSubscriberField = new OptionalFactoryField<>("externalPelvisPoseSubscriberField");

   private final OptionalFactoryField<RealtimeRos2Node> realtimeRos2NodeField = new OptionalFactoryField<>("realtimeRos2Node");
   private final OptionalFactoryField<MessageTopicNameGenerator> publisherTopicNameGeneratorField = new OptionalFactoryField<>("publisherTopicNameGenerator");
   private final OptionalFactoryField<MessageTopicNameGenerator> subscriberTopicNameGeneratorField = new OptionalFactoryField<>("subscriberTopicNameGenerator");

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
   private final OptionalFactoryField<ForceSensorDefinition[]> forceSensorDefinitionsField = new OptionalFactoryField<>("forceSensorDefinitionsField");
   private final OptionalFactoryField<IMUDefinition[]> imuDefinitionsField = new OptionalFactoryField<>("imuDefinitions");

   private final OptionalFactoryField<ContactableBodiesFactory<RobotSide>> contactableBodiesFactoryField = new OptionalFactoryField<>("contactableBodiesFactory");

   private final OptionalFactoryField<SensorReader> sensorReaderField = new OptionalFactoryField<>("sensorReader");
   private final OptionalFactoryField<SensorOutputMapReadOnly> rawSensorOutputMapField = new OptionalFactoryField<>("rawSensorOutputMap");
   private final OptionalFactoryField<SensorOutputMapReadOnly> processedSensorOutputMapField = new OptionalFactoryField<>("processedSensorOutputMap");
   private final OptionalFactoryField<ForceSensorStateUpdater> forceSensorStateUpdaterField = new OptionalFactoryField<>("forceSensorStateUpdater");

   private final OptionalFactoryField<JointDesiredOutputWriter> jointDesiredOutputWriterField = new OptionalFactoryField<>("jointDesiredOutputWriter");

   /**
    * Creates a new factory to create {@link AvatarEstimatorThread}.
    * <p>
    * Example for configuring this factory:
    *
    * <pre>
    * AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
    * avatarEstimatorThreadFactory.setROS2Info(realtimeRos2Node, robotName);
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
    * @param robotModel the robot model used to configure this factory.
    */
   public void configureWithDRCRobotModel(DRCRobotModel robotModel)
   {
      configureWithWholeBodyControllerParameters(robotModel);
      setEstimatorFullRobotModel(robotModel.createFullRobotModel());
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
    *                  {@link ControllerAPIDefinition#getPublisherTopicNameGenerator(String)} and
    *                  {@link ControllerAPIDefinition#getSubscriberTopicNameGenerator(String)}.
    */
   public void setROS2Info(RealtimeRos2Node ros2Node, String robotName)
   {
      setROS2Info(ros2Node,
                  ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                  ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));
   }

   /**
    * ROS 2 necessary information to create the real-time publisher/subscriber.
    *
    * @param ros2Node                     the real-time node to create the publisher with.
    * @param publisherTopicNameGenerator  the generator to use for creating the topic name for
    *                                     publishers.
    * @param subscriberTopicNameGenerator the generator to use for creating the topic name for
    *                                     subscribers.
    */
   public void setROS2Info(RealtimeRos2Node ros2Node, MessageTopicNameGenerator publisherTopicNameGenerator,
                           MessageTopicNameGenerator subscriberTopicNameGenerator)
   {
      realtimeRos2NodeField.set(ros2Node);
      publisherTopicNameGeneratorField.set(publisherTopicNameGenerator);
      subscriberTopicNameGeneratorField.set(subscriberTopicNameGenerator);
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
    * {@link ParameterLoaderHelper#loadParameters(Object, WholeBodyControllerParameters, YoVariableRegistry)}.
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
         throw new IllegalArgumentException("The main state estimator has already been set.");
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
         throw new IllegalArgumentException("Cannot add state estimator because SensorReaderFactory.useStateEstimator() is false.");
      getSecondaryStateEstimators().add(reinitilizeSupplier, secondaryStateEstimator);
   }

   public AvatarEstimatorThread createAvatarEstimatorThread()
   {
      if (jointDesiredOutputWriterField.hasValue())
      {
         jointDesiredOutputWriterField.get().setJointDesiredOutputList(getDesiredJointDataHolder());
         getEstimatorRegistry().addChild(jointDesiredOutputWriterField.get().getYoVariableRegistry());
      }

      AvatarEstimatorThread avatarEstimatorThread = new AvatarEstimatorThread(getSensorReader(),
                                                                              getEstimatorFullRobotModel(),
                                                                              getHumanoidRobotContextData(),
                                                                              getMainStateEstimator(),
                                                                              getSecondaryStateEstimators(),
                                                                              getForceSensorStateUpdater(),
                                                                              createControllerCrashPublisher(),
                                                                              getEstimatorRegistry(),
                                                                              getYoGraphicsListRegistry());
      avatarEstimatorThread.setRawOutputWriter(getRobotConfigurationDataPublisher());
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
      estimatorFactory.setCenterOfPressureDataHolderFromController(getCenterOfPressureDataHolderFromController());
      estimatorFactory.setRobotMotionStatusFromController(getRobotMotionStatusFromController());
      estimatorFactory.setExternalPelvisCorrectorSubscriber(getExternalPelvisPoseSubscriberField());
      return estimatorFactory.createStateEstimator(getEstimatorRegistry(), getYoGraphicsListRegistry());
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
      ParameterLoaderHelper.loadParameters(this, ekfParameterStream, ekfStateEstimator.getYoVariableRegistry());
      return ekfStateEstimator;
   }

   public RealtimeRos2Node getRealtimeRos2Node()
   {
      if (realtimeRos2NodeField.hasValue())
         return realtimeRos2NodeField.get();
      else
         return null;
   }

   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      if (publisherTopicNameGeneratorField.hasValue())
         return publisherTopicNameGeneratorField.get();
      else
         return null;
   }

   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      if (subscriberTopicNameGeneratorField.hasValue())
         return subscriberTopicNameGeneratorField.get();
      else
         return null;
   }

   public ForceSensorStateUpdater getForceSensorStateUpdater()
   {
      if (!useStateEstimator())
         return null;
      if (!forceSensorStateUpdaterField.hasValue())
      {
         // Updates the force sensor data when running with the estimator.
         forceSensorStateUpdaterField.set(new ForceSensorStateUpdater(getRootJoint(),
                                                                      getProcessedSensorOutputMap(),
                                                                      getForceSensorDataHolder(),
                                                                      stateEstimatorParametersField.get(),
                                                                      getGravity(),
                                                                      getRobotMotionStatusFromController(),
                                                                      getYoGraphicsListRegistry(),
                                                                      getEstimatorRegistry()));

         if (realtimeRos2NodeField.hasValue())
         {
            RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = new RequestWristForceSensorCalibrationSubscriber();
            ROS2Tools.createCallbackSubscription(realtimeRos2NodeField.get(),
                                                 RequestWristForceSensorCalibrationPacket.class,
                                                 subscriberTopicNameGeneratorField.get(),
                                                 subscriber -> requestWristForceSensorCalibrationSubscriber.receivedPacket(subscriber.takeNextData()));
            forceSensorStateUpdaterField.get().setRequestWristForceSensorCalibrationSubscriber(requestWristForceSensorCalibrationSubscriber);
         }
      }
      return forceSensorStateUpdaterField.get();
   }

   private IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> createControllerCrashPublisher()
   {
      if (realtimeRos2NodeField.hasValue())
         return ROS2Tools.createPublisher(realtimeRos2NodeField.get(), ControllerCrashNotificationPacket.class, publisherTopicNameGeneratorField.get());
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

   public RobotConfigurationDataPublisher getRobotConfigurationDataPublisher()
   {
      if (!realtimeRos2NodeField.hasValue())
         return null;

      if (!robotConfigurationDataPublisherField.hasValue())
      {
         ForceSensorDataHolderReadOnly forceSensorDataHolderToSend = getForceSensorDataHolder();
         if (getForceSensorStateUpdater() != null && getForceSensorStateUpdater().getForceSensorOutputWithGravityCancelled() != null)
            forceSensorDataHolderToSend = getForceSensorStateUpdater().getForceSensorOutputWithGravityCancelled();

         RobotConfigurationDataPublisherFactory factory = new RobotConfigurationDataPublisherFactory();
         factory.setDefinitionsToPublish(getEstimatorFullRobotModel());
         factory.setSensorSource(getEstimatorFullRobotModel(), forceSensorDataHolderToSend, getRawSensorOutputMap());
         factory.setRobotMotionStatusHolder(getRobotMotionStatusFromController());
         factory.setROS2Info(realtimeRos2NodeField.get(), publisherTopicNameGeneratorField.get());
         factory.setPublishPeriod(Conversions.secondsToNanoseconds(UnitConversions.hertzToSeconds(120)));
         robotConfigurationDataPublisherField.set(factory.createRobotConfigurationDataPublisher());
      }
      return robotConfigurationDataPublisherField.get();
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoVariableRegistry getEstimatorRegistry()
   {
      return estimatorRegistry;
   }
}
