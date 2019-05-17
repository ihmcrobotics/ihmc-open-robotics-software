package us.ihmc.avatar;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.ros.RosCore;
import org.ros.internal.message.Message;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.rosAPI.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.AvatarHumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationConstructionSetTools.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.robotController.SingleThreadedRobotController;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.SingleThreadedThreadDataSynchronizer;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class IHMCROSAPIPacketTest implements MultiRobotTestInterface
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private BlockingSimulationRunner blockingSimulationRunner;
   private RealtimeRos2Node realtimeRos2Node;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, "ihmc_ros_api_test");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (realtimeRos2Node != null)
      {
         realtimeRos2Node.stopSpinning();
         realtimeRos2Node = null;
      }

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   //TODO: Get rid of the stuff below and use a test helper.....

   @AfterEach
   public void destroyOtherStuff()
   {
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
         avatarSimulation = null;
      }
   }

   @Test
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
      DRCSimulationOutputWriterForControllerThread outputWriter = new DRCSimulationOutputWriterForControllerThread(sdfRobot);
      HumanoidGlobalDataProducer globalDataProducer = new HumanoidGlobalDataProducer(controllerCommunicatorServer);

      AbstractThreadedRobotController robotController = createController(robotModel, controllerCommunicatorServer, globalDataProducer, outputWriter, sdfRobot);
      sdfRobot.setController(robotController);

      OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDegreeOfFreedomJoints();

      robotController.doControl();
      controllerCommunicatorServer.send(HumanoidMessageTools.createHighLevelStateMessage(WALKING));

      new UiPacketToRosMsgRedirector(robotModel, rosUri, rosAPI_communicator_server, packetRouter, "/ihmc_ros/atlas");

      try
      {
         SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
         String nameSpace = "/ihmc_ros/atlas";
         String tfPrefix = null;
         new ThePeoplesGloriousNetworkProcessor(rosUri, rosAPI_communicator_server, null, ppsOffsetProvider, robotModel, nameSpace, tfPrefix,
                                                Collections.<Class> emptySet());
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
            ThreadTools.sleep((long) Conversions.secondsToMilliseconds(180));
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

      controllerCommunicatorClient.disconnect();
      controllerCommunicatorServer.disconnect();
      rosAPI_communicator_client.disconnect();
      rosAPI_communicator_server.disconnect();
   }

   private AvatarSimulation avatarSimulation;

   @Test
   public void testFuzzyPacketsWithoutRos()
   {
      DRCRobotModel robotModel = getRobotModel();
      Random random = new Random();

      PacketCommunicator packetCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT,
                                                                                                            new IHMCCommunicationKryoNetClassList());
      PacketCommunicator packetCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT,
                                                                                                            new IHMCCommunicationKryoNetClassList());

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
      DRCSimulationOutputWriterForControllerThread outputWriter = new DRCSimulationOutputWriterForControllerThread(sdfRobot);
      HumanoidGlobalDataProducer globalDataProducer = new HumanoidGlobalDataProducer(packetCommunicatorServer);

      AbstractThreadedRobotController robotController = createController(robotModel, packetCommunicatorServer, globalDataProducer, outputWriter, sdfRobot);
      sdfRobot.setController(robotController);

      OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDegreeOfFreedomJoints();

      robotController.doControl();
      packetCommunicatorClient.send(HumanoidMessageTools.createHighLevelStateMessage(WALKING));

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

      packetCommunicatorClient.disconnect();
      packetCommunicatorServer.disconnect();
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
                                                            HumanoidGlobalDataProducer dataProducer, DRCSimulationOutputWriterForControllerThread outputWriter,
                                                            FloatingRootJointRobot sdfRobot)
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      double gravity = -9.7925;

      HighLevelHumanoidControllerFactory controllerFactory = createHighLevelHumanoidControllerFactory(robotModel, packetCommunicator);
      RobotVisualizer robotVisualizer = null;
      ThreadDataSynchronizerInterface threadDataSynchronizer = new SingleThreadedThreadDataSynchronizer(null, robotModel, registry);

      Robot robot = new Robot("Robot");
      SingleThreadedRobotController robotController = new SingleThreadedRobotController("testSingleThreadedRobotController", robot, null);
      int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
      int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

      DRCPerfectSensorReaderFactory sensorReaderFactory = new DRCPerfectSensorReaderFactory(sdfRobot,
                                                                                            threadDataSynchronizer.getEstimatorForceSensorDataHolder(),
                                                                                            robotModel.getEstimatorDT());

      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSimpleRobotName(), robotModel.getSensorInformation(),
                                                                  robotModel.getContactPointParameters(), robotModel, robotModel.getStateEstimatorParameters(),
                                                                  sensorReaderFactory, threadDataSynchronizer, realtimeRos2Node, null, null, robotVisualizer,
                                                                  gravity);

//      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer,
//                                                                     outputWriter, dataProducer, robotVisualizer, gravity, robotModel.getEstimatorDT());
//
//      robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
//      robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);

      return robotController;
   }

   private HighLevelHumanoidControllerFactory createHighLevelHumanoidControllerFactory(DRCRobotModel robotModel, PacketCommunicator packetCommunicator)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();

      AvatarHumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames,
                                                                                                    feetContactSensorNames, wristForceSensorNames,
                                                                                                    highLevelControllerParameters, walkingControllerParameters,
                                                                                                    capturePointPlannerParameters);

      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeRos2Node);

      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);

      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      return controllerFactory;

   }
}
