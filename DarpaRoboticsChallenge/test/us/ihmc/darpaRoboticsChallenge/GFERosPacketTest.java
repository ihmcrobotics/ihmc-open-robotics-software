package us.ihmc.darpaRoboticsChallenge;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Random;

import org.apache.poi.ss.formula.functions.T;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
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
import us.ihmc.communication.packetCommunicator.LocalPacketCommunicator;
import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
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
   

   private DRCSimulationFactory drcSimulation;
   
   @Test
   public void testFuzzyPackets()
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
			   if(j % 5000 == 0)
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