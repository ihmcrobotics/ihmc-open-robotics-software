package us.ihmc.avatar;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.RosCore;
import org.ros.internal.message.Message;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.rosAPI.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationconstructionset.robotController.SingleThreadedRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.concurrent.SingleThreadedThreadDataSynchronizer;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public abstract class IHMCROSAPIPacketTest implements MultiRobotTestInterface
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private BlockingSimulationRunner blockingSimulationRunner;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   //TODO: Get rid of the stuff below and use a test helper.....

   @After
   public void destroyOtherStuff()
   {
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
         avatarSimulation = null;
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 8.7)
   @Test(timeout = 43000)
   public void testFuzzyPacketsUsingRos()
   {
      RosCore rosCore = RosCore.newPrivate();
      rosCore.start();
      URI rosUri = rosCore.getUri();
      System.out.println(rosUri);
      ThreadTools.sleep(2000);

      DRCRobotModel robotModel = getRobotModel();
      Random random = new Random();

      PacketCommunicator controllerCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
            new IHMCCommunicationKryoNetClassList());
      PacketCommunicator controllerCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
            new IHMCCommunicationKryoNetClassList());

      PacketCommunicator rosAPI_communicator_server = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR,
            new IHMCCommunicationKryoNetClassList());
      PacketCommunicator rosAPI_communicator_client = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR,
            new IHMCCommunicationKryoNetClassList());

      try
      {
         controllerCommunicatorServer.connect();
         controllerCommunicatorClient.connect();

         rosAPI_communicator_server.connect();
         rosAPI_communicator_client.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      PacketRouter<PacketDestination> packetRouter = new PacketRouter<>(PacketDestination.class);
      packetRouter.attachPacketCommunicator(PacketDestination.ROS_API, rosAPI_communicator_client);
      packetRouter.attachPacketCommunicator(PacketDestination.CONTROLLER, controllerCommunicatorClient);

      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      robotInitialSetup.initializeRobot(sdfRobot, robotModel.getJointMap());
      DRCSimulationOutputWriter outputWriter = new DRCSimulationOutputWriter(sdfRobot);
      HumanoidGlobalDataProducer globalDataProducer = new HumanoidGlobalDataProducer(controllerCommunicatorServer);

      AbstractThreadedRobotController robotController = createController(robotModel, controllerCommunicatorServer, globalDataProducer, outputWriter, sdfRobot);
      sdfRobot.setController(robotController);

      OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDegreeOfFreedomJoints();

      robotController.doControl();
      controllerCommunicatorServer.send(new HighLevelStateMessage(HighLevelState.WALKING));

      new UiPacketToRosMsgRedirector(robotModel, rosUri, rosAPI_communicator_server, packetRouter, "/ihmc_ros/atlas");

      try
      {
         SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
         String nameSpace = "/ihmc_ros/atlas";
         String tfPrefix = null;
         new ThePeoplesGloriousNetworkProcessor(rosUri, rosAPI_communicator_server, null, ppsOffsetProvider, robotModel, nameSpace, tfPrefix, Collections.<Class>emptySet());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      for (int i = 0; i < 100; i++)
      {
         robotController.doControl();
      }

      ArrayList<Class<?>> rosAPIPacketList = new ArrayList<>();
      rosAPIPacketList.addAll(GenericROSTranslationTools.getAllRosMessagePacketAnnotatedClasses());

      int randomModulus = random.nextInt(250) + 1;

      int iteration = 0;
      final AtomicBoolean timeNotElapsed = new AtomicBoolean(true);
      Runnable timer = new Runnable()
      {
         @Override
         public void run()
         {
            ThreadTools.sleep(TimeTools.secondsToMilliSeconds(180));
            timeNotElapsed.set(false);
         }
      };
      Thread t1 = new Thread(timer);
      t1.start();

      while (timeNotElapsed.get() && iteration < 1000000)
      {
         robotController.doControl();
         if (iteration % randomModulus == 0)
         {
            randomModulus = random.nextInt(250) + 1;
            int randomIndex = random.nextInt(rosAPIPacketList.size());
            Class randomClazz = rosAPIPacketList.get(randomIndex);

            Packet randomPacket = createRandomPacket(randomClazz, random);
            System.out.println(randomPacket.getClass() + " " + randomPacket);
            rosAPI_communicator_server.send(randomPacket);
         }
         iteration++;
      }

      controllerCommunicatorClient.close();
      controllerCommunicatorServer.close();
      rosAPI_communicator_client.close();
      rosAPI_communicator_server.close();
   }

   private AvatarSimulation avatarSimulation;

   @ContinuousIntegrationTest(estimatedDuration = 2.7)
   @Test(timeout = 30000)
   public void testFuzzyPacketsWithoutRos()
   {
      DRCRobotModel robotModel = getRobotModel();
      Random random = new Random();

      PacketCommunicator packetCommunicatorServer = PacketCommunicator
            .createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT, new IHMCCommunicationKryoNetClassList());
      PacketCommunicator packetCommunicatorClient = PacketCommunicator
            .createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT, new IHMCCommunicationKryoNetClassList());

      try
      {
         packetCommunicatorServer.connect();
         packetCommunicatorClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      robotInitialSetup.initializeRobot(sdfRobot, robotModel.getJointMap());
      DRCSimulationOutputWriter outputWriter = new DRCSimulationOutputWriter(sdfRobot);
      HumanoidGlobalDataProducer globalDataProducer = new HumanoidGlobalDataProducer(packetCommunicatorServer);

      AbstractThreadedRobotController robotController = createController(robotModel, packetCommunicatorServer, globalDataProducer, outputWriter, sdfRobot);
      sdfRobot.setController(robotController);

      OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDegreeOfFreedomJoints();

      robotController.doControl();
      packetCommunicatorClient.send(new HighLevelStateMessage(HighLevelState.WALKING));

      for (int i = 0; i < 100; i++)
      {
         robotController.doControl();
      }

      ArrayList<Class<?>> inputTopics = new ArrayList<>();
      inputTopics.addAll(GenericROSTranslationTools.getCoreInputTopics());
      for (int i = 0; i < inputTopics.size(); i++)
      {
         for (int j = 0; j < 100000; j++)
         {
            robotController.doControl();
            if (j % 300 == 0)
            {
               Packet randomPacket = createRandomPacket((Class<? extends Packet>) inputTopics.get(i), random);
               System.out.println(randomPacket);
               packetCommunicatorClient.send(randomPacket);
            }
            //			   if(j % 60 == 0)
            {
               //				   for(int k = 0; k < joints.length; k++)
               //				   {
               //					   System.out.println(joints[3].getTau());
               //				   }
            }
         }
      }

      packetCommunicatorClient.close();
      packetCommunicatorServer.close();
   }

   private Packet createRandomPacket(Class<? extends Packet> clazz, Random random)
   {
      Packet packet = null;
      Message translatedMessage;
      try
      {
         Constructor constructor = clazz.getDeclaredConstructor(Random.class);
         packet = (Packet) constructor.newInstance(random);
         return packet;
      }
      catch (InstantiationException | IllegalAccessException | NoSuchMethodException | InvocationTargetException e)
      {
         return null;
      }
   }

   private AbstractThreadedRobotController createController(DRCRobotModel robotModel, PacketCommunicator packetCommunicator,
         HumanoidGlobalDataProducer dataProducer, DRCSimulationOutputWriter outputWriter, FloatingRootJointRobot sdfRobot)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      double gravity = -9.7925;

      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, packetCommunicator);
      RobotVisualizer robotVisualizer = null;
      ThreadDataSynchronizerInterface threadDataSynchronizer = new SingleThreadedThreadDataSynchronizer(null, robotModel, registry);

      Robot robot = new Robot("Robot");
      SingleThreadedRobotController robotController = new SingleThreadedRobotController("testSingleThreadedRobotController", robot, null);
      int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
      int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

      DRCPerfectSensorReaderFactory sensorReaderFactory = new DRCPerfectSensorReaderFactory(sdfRobot,
            threadDataSynchronizer.getEstimatorForceSensorDataHolder(), robotModel.getEstimatorDT());

      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(),
            robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer,
            new PeriodicNonRealtimeThreadScheduler("DRCPoseCommunicator"), dataProducer, null, gravity);

      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer,
            outputWriter, dataProducer, robotVisualizer, gravity, robotModel.getEstimatorDT());

      robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
      robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);

      return robotController;
   }

   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, PacketCommunicator packetCommunicator)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      ICPOptimizationParameters icpOptimizationParameters = robotModel.getICPOptimizationParameters();
      final HighLevelState initialBehavior;
      initialBehavior = HighLevelState.DO_NOTHING_BEHAVIOR; // HERE!!

      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters,
            initialBehavior);
      controllerFactory.setICPOptimizationControllerParameters(icpOptimizationParameters);

      controllerFactory.createControllerNetworkSubscriber(new PeriodicNonRealtimeThreadScheduler("CapturabilityBasedStatusProducer"), packetCommunicator);

      return controllerFactory;
   }

}
