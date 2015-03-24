package us.ihmc.darpaRoboticsChallenge;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URI;
import java.util.Random;

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
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.LocalPacketCommunicator;
import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.AbstractThreadedRobotController;
import us.ihmc.simulationconstructionset.robotController.SingleThreadedRobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCSimulationOutputWriter;
import us.ihmc.wholeBodyController.concurrent.SingleThreadedThreadDataSynchronizer;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
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
      
      LocalPacketCommunicator controllerCommunicator = new LocalPacketCommunicator(PacketDestination.CONTROLLER.ordinal(),"controller");
      KryoLocalPacketCommunicator gfe_communicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.GFE.ordinal(), "GFE_Communicator");
      
      PacketRouter packetRouter = new PacketRouter();
      packetRouter.attachPacketCommunicator(gfe_communicator);
      packetRouter.attachPacketCommunicator(controllerCommunicator);
      
      SDFRobot sdfRobot = robotModel.createSdfRobot(false);
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      robotInitialSetup.initializeRobot(sdfRobot, robotModel.getJointMap());
      DRCSimulationOutputWriter outputWriter = new DRCSimulationOutputWriter(sdfRobot);
      GlobalDataProducer globalDataProducer = new GlobalDataProducer(controllerCommunicator);
      
      AbstractThreadedRobotController robotController = createController(robotModel, globalDataProducer, outputWriter, sdfRobot);
      sdfRobot.setController(robotController);
      
      OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDoFJoints();
      
      robotController.doControl();
      controllerCommunicator.send(new HighLevelStatePacket(HighLevelState.WALKING));
      
      new UiPacketToRosMsgRedirector(robotModel, rosUri, gfe_communicator, packetRouter);
      
      try
      {
         SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
         String nameSpace = "/ihmc_ros/atlas";
         new ThePeoplesGloriousNetworkProcessor(rosUri, gfe_communicator, null, ppsOffsetProvider, robotModel, nameSpace);
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
      for(int i = 0; i < 1000000; i++)
      {
         robotController.doControl();
         if(i % randomModulus == 0)
         {
            randomModulus = random.nextInt(250) + 1;
            int randomIndex = random.nextInt(gfePacketList.length);
            Class randomClazz = gfePacketList[randomIndex];
            
            Packet randomPacket = createRandomPacket(randomClazz, random);
            System.out.println(randomPacket.getClass() + " " + randomPacket);
            gfe_communicator.receivedPacket(randomPacket);
         }
      }
   }

   private DRCSimulationFactory drcSimulation;
   
   @Test
   public void testFuzzyPacketsWithoutRos()
   {
	   DRCRobotModel robotModel = getRobotModel();
	   Random random = new Random();
	   
	   LocalPacketCommunicator packetCommunicator = new LocalPacketCommunicator(PacketDestination.CONTROLLER.ordinal(),"controller");
	   SDFRobot sdfRobot = robotModel.createSdfRobot(false);
	   DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
	   robotInitialSetup.initializeRobot(sdfRobot, robotModel.getJointMap());
	   DRCSimulationOutputWriter outputWriter = new DRCSimulationOutputWriter(sdfRobot);
	   GlobalDataProducer globalDataProducer = new GlobalDataProducer(packetCommunicator);
	   
	   AbstractThreadedRobotController robotController = createController(robotModel, globalDataProducer, outputWriter, sdfRobot);
	   sdfRobot.setController(robotController);
	   
	   
	   OneDegreeOfFreedomJoint[] joints = sdfRobot.getOneDoFJoints();
	   
	   robotController.doControl();
	   packetCommunicator.send(new HighLevelStatePacket(HighLevelState.WALKING));
	   
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
				   packetCommunicator.send(randomPacket);
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
	   
	   DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", registry);
	   SingleThreadedRobotController robotController = new SingleThreadedRobotController("testSingleThreadedRobotController", yoTime, null );
	   int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
	   int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());
	   
	   
	   DRCPerfectSensorReaderFactory sensorReaderFactory = new DRCPerfectSensorReaderFactory(sdfRobot, robotModel.getEstimatorDT());
	   
	   DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(), robotModel.getStateEstimatorParameters(),
	    		  sensorReaderFactory, threadDataSynchronizer, dataProducer, null, gravity);
	   
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


      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
    		  robotModel.getSensorInformation().getFeetForceSensorNames(), walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, initialBehavior);

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParameters);
      VariousWalkingProviderFactory variousWalkingProviderFactory;
      variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters);
      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);

      return controllerFactory;
   }
   
   
   
   
}