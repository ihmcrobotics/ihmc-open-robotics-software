package us.ihmc.darpaRoboticsChallenge;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import org.apache.poi.ss.formula.functions.T;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.RosCore;
import org.ros.internal.message.Message;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DataProducerVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationconstructionset.robotController.SingleThreadedRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;
import us.ihmc.tools.time.TimeTools;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCSimulationOutputWriter;
import us.ihmc.wholeBodyController.concurrent.SingleThreadedThreadDataSynchronizer;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class GFERosPacketTest implements MultiRobotTestInterface
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
      if (drcSimulation != null)
      {
         drcSimulation.dispose();
         drcSimulation = null;
      }
   }
   
   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 30000)
   public void testFuzzyPacketsUsingRos()
   {
      RosCore rosCore = RosCore.newPrivate();
      rosCore.start();
      URI rosUri = rosCore.getUri();
      System.out.println(rosUri);
      ThreadTools.sleep(2000);
      
      DRCRobotModel robotModel = getRobotModel();
      Random random = new Random();
      
      PacketCommunicator controllerCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      PacketCommunicator controllerCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      
      PacketCommunicator gfe_communicator_server = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
      PacketCommunicator gfe_communicator_client = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
      
      
      try
      {
         controllerCommunicatorServer.connect();
         controllerCommunicatorClient.connect();
         
         gfe_communicator_server.connect();
         gfe_communicator_client.connect();
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
      
      
      PacketRouter<PacketDestination> packetRouter = new PacketRouter<>(PacketDestination.class);
      packetRouter.attachPacketCommunicator(PacketDestination.GFE, gfe_communicator_client);
      packetRouter.attachPacketCommunicator(PacketDestination.CONTROLLER, controllerCommunicatorClient);
      
      SDFRobot sdfRobot = robotModel.createSdfRobot(false);
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      robotInitialSetup.initializeRobot(sdfRobot, robotModel.getJointMap());
      DRCSimulationOutputWriter outputWriter = new DRCSimulationOutputWriter(sdfRobot);
      GlobalDataProducer globalDataProducer = new GlobalDataProducer(controllerCommunicatorServer);
      
      AbstractThreadedRobotController robotController = createController(robotModel, globalDataProducer, outputWriter, sdfRobot);
      sdfRobot.setController(robotController);
      
      OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDoFJoints();
      
      robotController.doControl();
      controllerCommunicatorServer.send(new HighLevelStatePacket(HighLevelState.WALKING));
      
      new UiPacketToRosMsgRedirector(robotModel, rosUri, gfe_communicator_server, packetRouter, "/ihmc_ros/atlas");
      
      try
      {
         SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
         String nameSpace = "/ihmc_ros/atlas";
         String tfPrefix = null;
         new ThePeoplesGloriousNetworkProcessor(rosUri, gfe_communicator_server, null, ppsOffsetProvider, robotModel, nameSpace, tfPrefix);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
      for(int i = 0; i < 100; i++)
      {
         robotController.doControl();
      }
      
      Class[] gfePacketList = IHMCRosApiMessageMap.PACKET_LIST;
      
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
      
      while(timeNotElapsed .get() && iteration < 1000000)
      {
         robotController.doControl();
         if(iteration % randomModulus == 0)
         {
            randomModulus = random.nextInt(250) + 1;
            int randomIndex = random.nextInt(gfePacketList.length);
            Class randomClazz = gfePacketList[randomIndex];
            
            Packet randomPacket = createRandomPacket(randomClazz, random);
            System.out.println(randomPacket.getClass() + " " + randomPacket);
            gfe_communicator_server.send(randomPacket);
         }
         iteration++;
      }
      
      controllerCommunicatorClient.close();
      controllerCommunicatorServer.close();
      gfe_communicator_client.close();
      gfe_communicator_server.close();
   }

   private DRCSimulationFactory drcSimulation;
   
   @EstimatedDuration(duration = 1.0)
   @Test(timeout = 30000)
   public void testFuzzyPacketsWithoutRos()
   {
	   DRCRobotModel robotModel = getRobotModel();
	   Random random = new Random();
	   
	   
	   PacketCommunicator packetCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT, new IHMCCommunicationKryoNetClassList());
	   PacketCommunicator packetCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT, new IHMCCommunicationKryoNetClassList());
	   
	   try
      {
         packetCommunicatorServer.connect();
         packetCommunicatorClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
	   
	   SDFRobot sdfRobot = robotModel.createSdfRobot(false);
	   DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
	   robotInitialSetup.initializeRobot(sdfRobot, robotModel.getJointMap());
	   DRCSimulationOutputWriter outputWriter = new DRCSimulationOutputWriter(sdfRobot);
	   GlobalDataProducer globalDataProducer = new GlobalDataProducer(packetCommunicatorServer);
	   
	   AbstractThreadedRobotController robotController = createController(robotModel, globalDataProducer, outputWriter, sdfRobot);
	   sdfRobot.setController(robotController);
	   
	   
	   OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDoFJoints();
	   
	   robotController.doControl();
	   packetCommunicatorClient.send(new HighLevelStatePacket(HighLevelState.WALKING));
	   
	   for(int i = 0; i < 100; i++)
	   {
		   robotController.doControl();
	   }
	   
	   for (int i = 0; i < IHMCRosApiMessageMap.INPUT_PACKET_LIST.length; i++)
	   {
		   
		   for(int j = 0; j < 100000; j++)
		   {
			   robotController.doControl();
			   if(j % 300 == 0)
			   {
				   Packet randomPacket = createRandomPacket(IHMCRosApiMessageMap.INPUT_PACKET_LIST[i], random);
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
	   
   private Packet createRandomPacket(Class<T> clazz, Random random)
   {
	   Packet packet = null;
	   Message translatedMessage;
	   T translated = null;
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
   
   private AbstractThreadedRobotController createController(DRCRobotModel robotModel, GlobalDataProducer dataProducer, DRCSimulationOutputWriter outputWriter, SDFRobot sdfRobot)
   {
	   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
	   double gravity = -9.7925;
	   
	   MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, dataProducer);
	   RobotVisualizer robotVisualizer = null;
	   ThreadDataSynchronizerInterface threadDataSynchronizer = new SingleThreadedThreadDataSynchronizer(null, robotModel, registry);
	   
	   Robot robot = new Robot("Robot");
	   SingleThreadedRobotController robotController = new SingleThreadedRobotController("testSingleThreadedRobotController", robot, null );
	   int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
	   int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());
	   
	   
	   DRCPerfectSensorReaderFactory sensorReaderFactory = new DRCPerfectSensorReaderFactory(sdfRobot, threadDataSynchronizer.getEstimatorForceSensorDataHolder(), robotModel.getEstimatorDT());
	   
	   DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(), robotModel.getStateEstimatorParameters(),
	    		  sensorReaderFactory, threadDataSynchronizer, new PeriodicNonRealtimeThreadScheduler("DRCPoseCommunicator"), dataProducer, null, gravity);
	   
	   DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer, outputWriter, dataProducer,
			   robotVisualizer, gravity, robotModel.getEstimatorDT());
	   
	   robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
	   robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);
	   
	   return robotController;
   }
   
   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, GlobalDataProducer dataProducer)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      final HighLevelState initialBehavior;
      initialBehavior = HighLevelState.DO_NOTHING_BEHAVIOR; // HERE!!


      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, initialBehavior);

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParameters);
      VariousWalkingProviderFactory variousWalkingProviderFactory;
      variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters, new PeriodicNonRealtimeThreadScheduler("CapturabilityBasedStatusProducer"));
      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);

      return controllerFactory;
   }
   
   
   
   
}