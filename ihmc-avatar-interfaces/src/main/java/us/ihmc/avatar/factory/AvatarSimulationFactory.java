package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.sensors.DRCSimulatedIMUPublisher;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.simulationConstructionSetTools.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.robotController.SingleThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.ActualCMPComputer;
import us.ihmc.simulationToolkit.controllers.JointLowLevelJointControlSimulator;
import us.ihmc.simulationToolkit.controllers.PIDLidarTorqueController;
import us.ihmc.simulationToolkit.controllers.PassiveJointController;
import us.ihmc.simulationToolkit.controllers.SimulatedRobotCenterOfMassVisualizer;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithStateChangeSmoother;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithTorqueOffsets;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.concurrent.SingleThreadedThreadDataSynchronizer;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AvatarSimulationFactory
{
   private final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   private final RequiredFactoryField<HighLevelHumanoidControllerFactory> highLevelHumanoidControllerFactory = new RequiredFactoryField<>("highLevelHumanoidControllerFactory");
   private final RequiredFactoryField<CommonAvatarEnvironmentInterface> commonAvatarEnvironment = new RequiredFactoryField<>("commonAvatarEnvironmentInterface");
   private final RequiredFactoryField<DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>> robotInitialSetup = new RequiredFactoryField<>("robotInitialSetup");
   private final RequiredFactoryField<DRCSCSInitialSetup> scsInitialSetup = new RequiredFactoryField<>("scsInitialSetup");
   private final RequiredFactoryField<DRCGuiInitialSetup> guiInitialSetup = new RequiredFactoryField<>("guiInitialSetup");
   private final RequiredFactoryField<HumanoidGlobalDataProducer> humanoidGlobalDataProducer = new RequiredFactoryField<>("humanoidGlobalDataProducer");

   private final OptionalFactoryField<Double> gravity = new OptionalFactoryField<>("gravity");
   private final OptionalFactoryField<Boolean> doSlowIntegrationForTorqueOffset = new OptionalFactoryField<>("doSlowIntegrationForTorqueOffset");
   private final OptionalFactoryField<Boolean> doSmoothJointTorquesAtControllerStateChanges = new OptionalFactoryField<>("doSmoothJointTorquesAtControllerStateChanges");
   private final OptionalFactoryField<Boolean> addActualCMPVisualization = new OptionalFactoryField<>("addActualCMPVisualization");
   private final OptionalFactoryField<Boolean> createCollisionMeshes = new OptionalFactoryField<>("createCollisionMeshes");

   // TO CONSTRUCT
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private YoVariableServer yoVariableServer;
   private SimulationConstructionSet simulationConstructionSet;
   private ThreadDataSynchronizerInterface threadDataSynchronizer;
   private SensorReaderFactory sensorReaderFactory;
   private JointDesiredOutputWriter simulationOutputWriter;
   private DRCOutputProcessor simulationOutputProcessor;
   private DRCEstimatorThread stateEstimationThread;
   private DRCControllerThread controllerThread;
   private AbstractThreadedRobotController threadedRobotController;
   private CloseableAndDisposableRegistry closeableAndDisposableRegistry;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private ActualCMPComputer actualCMPComputer;

   private void createHumanoidFloatingRootJointRobot()
   {
      humanoidFloatingRootJointRobot = robotModel.get().createHumanoidFloatingRootJointRobot(createCollisionMeshes.get());
   }

   private void setupYoVariableServer()
   {
      if (robotModel.get().getLogSettings().isLog())
      {
         yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(),
                                                 robotModel.get().getLogModelProvider(), robotModel.get().getLogSettings(), robotModel.get().getEstimatorDT());
      }
      else
      {
         yoVariableServer = null;
      }
   }

   private void setupSimulationConstructionSet()
   {
      SimulationConstructionSetParameters simulationConstructionSetParameters = guiInitialSetup.get().getSimulationConstructionSetParameters();
      simulationConstructionSetParameters.setDataBufferSize(scsInitialSetup.get().getSimulationDataBufferSize());

      List<Robot> allSimulatedRobotList = new ArrayList<Robot>();
      allSimulatedRobotList.add(humanoidFloatingRootJointRobot);
      if (commonAvatarEnvironment.get() != null && commonAvatarEnvironment.get().getEnvironmentRobots() != null)
      {
         allSimulatedRobotList.addAll(commonAvatarEnvironment.get().getEnvironmentRobots());

         commonAvatarEnvironment.get().addContactPoints(humanoidFloatingRootJointRobot.getAllGroundContactPoints());
         commonAvatarEnvironment.get().createAndSetContactControllerToARobot();
      }

      simulationConstructionSet = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), guiInitialSetup.get().getGraphics3DAdapter(),
                                                                simulationConstructionSetParameters);
      simulationConstructionSet.setDT(robotModel.get().getSimulateDT(), 1);
   }

   private void setupThreadDataSynchronizer()
   {
      if (scsInitialSetup.get().getRunMultiThreaded())
      {
         threadDataSynchronizer = new ThreadDataSynchronizer(robotModel.get());
      }
      else
      {
         YoVariableRegistry threadDataSynchronizerRegistry = new YoVariableRegistry("ThreadDataSynchronizerRegistry");
         threadDataSynchronizer = new SingleThreadedThreadDataSynchronizer(simulationConstructionSet, robotModel.get(), threadDataSynchronizerRegistry);
         simulationConstructionSet.addYoVariableRegistry(threadDataSynchronizerRegistry);
      }
   }

   private void setupSensorReaderFactory()
   {
      if (scsInitialSetup.get().usePerfectSensors())
      {
         sensorReaderFactory = new DRCPerfectSensorReaderFactory(humanoidFloatingRootJointRobot, threadDataSynchronizer.getEstimatorForceSensorDataHolder(),
                                                                 robotModel.get().getStateEstimatorParameters().getEstimatorDT());
      }
      else
      {
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(humanoidFloatingRootJointRobot,
                                                                                  robotModel.get().getStateEstimatorParameters());
      }
   }

   private void setupSimulationOutputWriter()
   {
      simulationOutputWriter = robotModel.get().getCustomSimulationOutputWriter(humanoidFloatingRootJointRobot);
   }

   private void setupSimulationOutputProcessor()
   {

      simulationOutputProcessor = robotModel.get().getCustomSimulationOutputProcessor(humanoidFloatingRootJointRobot);

      if (doSmoothJointTorquesAtControllerStateChanges.get())
      {
         DRCOutputProcessorWithStateChangeSmoother drcOutputWriterWithStateChangeSmoother = new DRCOutputProcessorWithStateChangeSmoother(simulationOutputProcessor);
         highLevelHumanoidControllerFactory.get().attachControllerStateChangedListener(drcOutputWriterWithStateChangeSmoother.createControllerStateChangedListener());

         simulationOutputProcessor = drcOutputWriterWithStateChangeSmoother;
      }

      if (doSlowIntegrationForTorqueOffset.get())
      {
         DRCOutputProcessorWithTorqueOffsets outputWriterWithTorqueOffsets = new DRCOutputProcessorWithTorqueOffsets(simulationOutputProcessor,
                                                                                                               robotModel.get().getControllerDT());
         simulationOutputProcessor = outputWriterWithTorqueOffsets;
      }
   }

   private void setupStateEstimationThread()
   {
      stateEstimationThread = new DRCEstimatorThread(robotModel.get().getSensorInformation(), robotModel.get().getContactPointParameters(),
                                                     robotModel.get(), robotModel.get().getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer,
                                                     new PeriodicNonRealtimeThreadScheduler("DRCSimGazeboYoVariableServer"), humanoidGlobalDataProducer.get(),
                                                     simulationOutputWriter, yoVariableServer, gravity.get());

      if (humanoidGlobalDataProducer.get() != null)
      {
         PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator = new PelvisPoseCorrectionCommunicator(humanoidGlobalDataProducer.get());
         humanoidGlobalDataProducer.get().attachListener(StampedPosePacket.class, pelvisPoseCorrectionCommunicator);
         stateEstimationThread.setExternalPelvisCorrectorSubscriber(pelvisPoseCorrectionCommunicator);
      }
      else
      {
         stateEstimationThread.setExternalPelvisCorrectorSubscriber(null);
      }
   }

   private void setupControllerThread()
   {
      controllerThread = new DRCControllerThread(robotModel.get(), robotModel.get().getSensorInformation(), highLevelHumanoidControllerFactory.get(),
                                                 threadDataSynchronizer, simulationOutputProcessor, humanoidGlobalDataProducer.get(), yoVariableServer,
                                                 gravity.get(), robotModel.get().getEstimatorDT());
   }

   private void createClosableAndDisposableRegistry()
   {
      closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();
   }

   private void setupMultiThreadedRobotController()
   {
      if (scsInitialSetup.get().getRunMultiThreaded())
      {
         threadedRobotController = new MultiThreadedRobotController("DRCSimulation", humanoidFloatingRootJointRobot, simulationConstructionSet);
      }
      else
      {
         PrintTools.warn(this, "Running simulation in single threaded mode", true);
         threadedRobotController = new SingleThreadedRobotController("DRCSimulation", humanoidFloatingRootJointRobot, simulationConstructionSet);
      }
      int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.get().getEstimatorDT() / robotModel.get().getSimulateDT());
      int controllerTicksPerSimulationTick = (int) Math.round(robotModel.get().getControllerDT() / robotModel.get().getSimulateDT());
      int slowPublisherTicksPerSimulationTick = (int) Math.round(10 * robotModel.get().getEstimatorDT() / robotModel.get().getSimulateDT());

      threadedRobotController.addController(stateEstimationThread, estimatorTicksPerSimulationTick, false);
      threadedRobotController.addController(controllerThread, controllerTicksPerSimulationTick, true);
      MultiThreadedRobotControlElement simulatedHandController = robotModel.get().createSimulatedHandController(humanoidFloatingRootJointRobot,
                                                                                                                threadDataSynchronizer,
                                                                                                                humanoidGlobalDataProducer.get(),
                                                                                                                closeableAndDisposableRegistry);
      if (simulatedHandController != null)
      {
         threadedRobotController.addController(simulatedHandController, controllerTicksPerSimulationTick, false);
      }
      DRCRobotJointMap jointMap = robotModel.get().getJointMap();
      if (jointMap.getHeadName() != null)
      {
         DRCSimulatedIMUPublisher drcSimulatedIMUPublisher = new DRCSimulatedIMUPublisher(humanoidGlobalDataProducer.get(),
                                                                                          stateEstimationThread.getSimulatedIMUOutput(),
                                                                                          jointMap.getHeadName());
         threadedRobotController.addController(drcSimulatedIMUPublisher, slowPublisherTicksPerSimulationTick, false);
      }
   }

   private void initializeStateEstimatorToActual()
   {
      if (scsInitialSetup.get().getInitializeEstimatorToActual())
      {
         PrintTools.info(this, "Initializing estimator to actual");

         /**
          * The following is to get the initial CoM position from the robot. It is cheating for now, and we need
          * to move to where the robot itself determines coordinates, and the sensors are all in the
          * robot-determined world coordinates..
          */
         robotInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get().getJointMap());
         try
         {
            humanoidFloatingRootJointRobot.update();
            humanoidFloatingRootJointRobot.doDynamicsButDoNotIntegrate();
            humanoidFloatingRootJointRobot.update();
         }
         catch (UnreasonableAccelerationException e)
         {
            throw new RuntimeException("UnreasonableAccelerationException");
         }

         Point3D initialCoMPosition = new Point3D();
         humanoidFloatingRootJointRobot.computeCenterOfMass(initialCoMPosition);

         Quaternion initialEstimationLinkOrientation = new Quaternion();
         humanoidFloatingRootJointRobot.getRootJoint().getJointTransform3D().getRotation(initialEstimationLinkOrientation);
         Vector3D initialRootVelocity = new Vector3D();
         humanoidFloatingRootJointRobot.getRootJoint().getVelocity(initialRootVelocity);
         stateEstimationThread.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation, initialRootVelocity, null);
      }
   }

   private void setupThreadedRobotController()
   {
      humanoidFloatingRootJointRobot.setController(threadedRobotController);
   }

   private void setupLidarController()
   {
      DRCRobotLidarParameters lidarParameters = robotModel.get().getSensorInformation().getLidarParameters(0);
      if (lidarParameters != null && lidarParameters.getLidarSpindleJointName() != null)
      {
         PIDLidarTorqueController pidLidarTorqueController = new PIDLidarTorqueController(humanoidFloatingRootJointRobot,
                                                                                          lidarParameters.getLidarSpindleJointName(),
                                                                                          lidarParameters.getLidarSpindleVelocity(),
                                                                                          robotModel.get().getSimulateDT());
         humanoidFloatingRootJointRobot.setController(pidLidarTorqueController);
      }
   }

   private void setupPositionControlledJointsForSimulation()
   {
      String[] positionControlledJointNames = robotModel.get().getJointMap().getPositionControlledJointsForSimulation();
      if (positionControlledJointNames != null)
      {
         for (String positionControlledJointName : positionControlledJointNames)
         {
            OneDegreeOfFreedomJoint simulatedJoint = humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoint(positionControlledJointName);
            FullRobotModel controllerFullRobotModel = threadDataSynchronizer.getControllerFullRobotModel();
            OneDoFJoint controllerJoint = controllerFullRobotModel.getOneDoFJointByName(positionControlledJointName);
            JointDesiredOutputList controllerLowLevelDataList = threadDataSynchronizer.getControllerDesiredJointDataHolder();
            JointDesiredOutput controllerDesiredOutput = controllerLowLevelDataList.getJointDesiredOutput(controllerJoint);

            JointRole jointRole = robotModel.get().getJointMap().getJointRole(positionControlledJointName);
            boolean isUpperBodyJoint = ((jointRole != JointRole.LEG) && (jointRole != JointRole.SPINE));
            boolean isBackJoint = jointRole == JointRole.SPINE;

            if (simulatedJoint == null || controllerJoint == null)
               continue;

            JointLowLevelJointControlSimulator positionControlSimulator = new JointLowLevelJointControlSimulator(simulatedJoint, controllerJoint,
                                                                                                                 controllerDesiredOutput, isUpperBodyJoint,
                                                                                                                 isBackJoint, false,
                                                                                                                 controllerFullRobotModel.getTotalMass(), robotModel.get().getSimulateDT());
            humanoidFloatingRootJointRobot.setController(positionControlSimulator);
         }
      }
   }

   private void setupPassiveJoints()
   {
      YoVariableRegistry robotsYoVariableRegistry = humanoidFloatingRootJointRobot.getRobotsYoVariableRegistry();
      List<ImmutablePair<String, YoPDGains>> passiveJointNameWithGains = robotModel.get().getJointMap().getPassiveJointNameWithGains(robotsYoVariableRegistry);
      if (passiveJointNameWithGains != null)
      {
         for (int i = 0; i < passiveJointNameWithGains.size(); i++)
         {
            String jointName = passiveJointNameWithGains.get(i).getLeft();
            OneDegreeOfFreedomJoint simulatedJoint = humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoint(jointName);
            YoPDGains gains = passiveJointNameWithGains.get(i).getRight();
            PassiveJointController passiveJointController = new PassiveJointController(simulatedJoint, gains);
            humanoidFloatingRootJointRobot.setController(passiveJointController);
         }
      }
   }

   private void setupSimulatedRobotTimeProvider()
   {
      simulatedRobotTimeProvider = new SimulatedDRCRobotTimeProvider(robotModel.get().getSimulateDT());
      humanoidFloatingRootJointRobot.setController(simulatedRobotTimeProvider);
   }

   private void setupCMPVisualization()
   {
      actualCMPComputer = new ActualCMPComputer(addActualCMPVisualization.get(), simulationConstructionSet, humanoidFloatingRootJointRobot);
      if (addActualCMPVisualization.get())
      {
         humanoidFloatingRootJointRobot.setController(actualCMPComputer);
      }
   }

   private void setupCOMVisualization()
   {
      SimulatedRobotCenterOfMassVisualizer simulatedRobotCenterOfMassVisualizer = new SimulatedRobotCenterOfMassVisualizer(humanoidFloatingRootJointRobot,
                                                                                                                           robotModel.get().getSimulateDT());
      humanoidFloatingRootJointRobot.setController(simulatedRobotCenterOfMassVisualizer);
   }

   private void initializeSimulationConstructionSet()
   {
      simulationConstructionSet.setParameterRootPath(threadedRobotController.getYoVariableRegistry());
      
      humanoidFloatingRootJointRobot.setDynamicIntegrationMethod(scsInitialSetup.get().getDynamicIntegrationMethod());
      scsInitialSetup.get().initializeSimulation(simulationConstructionSet);

      if (guiInitialSetup.get().isGuiShown())
      {
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = simulationConstructionSet.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.setShowOnStart(guiInitialSetup.get().isShowOverheadView());
         simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(controllerThread.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(stateEstimationThread.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(actualCMPComputer.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.createOverheadPlotter();
         guiInitialSetup.get().initializeGUI(simulationConstructionSet, humanoidFloatingRootJointRobot, robotModel.get());
      }

      if (commonAvatarEnvironment.get() != null && commonAvatarEnvironment.get().getTerrainObject3D() != null)
      {
         simulationConstructionSet.addStaticLinkGraphics(commonAvatarEnvironment.get().getTerrainObject3D().getLinkGraphics());
      }

      scsInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get(), null);
      robotInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get().getJointMap());
      humanoidFloatingRootJointRobot.update();
   }

   public AvatarSimulation createAvatarSimulation()
   {
      gravity.setDefaultValue(-9.81);
      doSlowIntegrationForTorqueOffset.setDefaultValue(false);
      doSmoothJointTorquesAtControllerStateChanges.setDefaultValue(false);
      addActualCMPVisualization.setDefaultValue(true);
      createCollisionMeshes.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      createHumanoidFloatingRootJointRobot();
      setupYoVariableServer();
      setupSimulationConstructionSet();
      setupThreadDataSynchronizer();
      setupSensorReaderFactory();
      setupSimulationOutputWriter();
      setupSimulationOutputProcessor();
      setupStateEstimationThread();
      setupControllerThread();
      createClosableAndDisposableRegistry();
      setupMultiThreadedRobotController();
      initializeStateEstimatorToActual();
      setupThreadedRobotController();
      setupLidarController();
      setupPositionControlledJointsForSimulation();
      setupPassiveJoints();
      setupSimulatedRobotTimeProvider();
      setupCMPVisualization();
      setupCOMVisualization();
      initializeSimulationConstructionSet();

      AvatarSimulation avatarSimulation = new AvatarSimulation();
      avatarSimulation.setSimulationConstructionSet(simulationConstructionSet);
      avatarSimulation.setHighLevelHumanoidControllerFactory(highLevelHumanoidControllerFactory.get());
      avatarSimulation.setYoVariableServer(yoVariableServer);
      avatarSimulation.setCloseableAndDisposableRegistry(closeableAndDisposableRegistry);
      avatarSimulation.setControllerThread(controllerThread);
      avatarSimulation.setStateEstimationThread(stateEstimationThread);
      avatarSimulation.setHumanoidGlobalDataProducer(humanoidGlobalDataProducer.get());
      avatarSimulation.setThreadedRobotController(threadedRobotController);
      avatarSimulation.setHumanoidFloatingRootJointRobot(humanoidFloatingRootJointRobot);
      avatarSimulation.setSimulatedRobotTimeProvider(simulatedRobotTimeProvider);
      avatarSimulation.setThreadDataSynchronizer(threadDataSynchronizer);

      FactoryTools.disposeFactory(this);

      return avatarSimulation;
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel.set(robotModel);
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this.highLevelHumanoidControllerFactory.set(highLevelHumanoidControllerFactory);
   }

   public void setCommonAvatarEnvironment(CommonAvatarEnvironmentInterface commonAvatarEnvironment)
   {
      this.commonAvatarEnvironment.set(commonAvatarEnvironment);
   }

   public void setRobotInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup.set(robotInitialSetup);
   }

   public void setSCSInitialSetup(DRCSCSInitialSetup scsInitialSetup)
   {
      this.scsInitialSetup.set(scsInitialSetup);
   }

   public void setGuiInitialSetup(DRCGuiInitialSetup guiInitialSetup)
   {
      this.guiInitialSetup.set(guiInitialSetup);
   }

   public void setHumanoidGlobalDataProducer(HumanoidGlobalDataProducer humanoidGlobalDataProducer)
   {
      this.humanoidGlobalDataProducer.set(humanoidGlobalDataProducer);
   }

   public void setDoSlowIntegrationForTorqueOffset(boolean doSlowIntegrationForTorqueOffset)
   {
      this.doSlowIntegrationForTorqueOffset.set(doSlowIntegrationForTorqueOffset);
   }

   public void setDoSmoothJointTorquesAtControllerStateChanges(boolean doSmoothJointTorquesAtControllerStateChanges)
   {
      this.doSmoothJointTorquesAtControllerStateChanges.set(doSmoothJointTorquesAtControllerStateChanges);
   }

   public void setAddActualCMPVisualization(boolean addActualCMPVisualization)
   {
      this.addActualCMPVisualization.set(addActualCMPVisualization);
   }

   public void setCreateCollisionMeshes(boolean createCollisionMeshes)
   {
      this.createCollisionMeshes.set(createCollisionMeshes);
   }

   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }
}
