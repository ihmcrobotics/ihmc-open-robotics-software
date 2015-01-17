package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.networking.CommandRecorder;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngineSettings;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptObject;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCScriptBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean DEBUG = true;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = true || createMovie;

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private ReferenceFrame currentScriptFrame = worldFrame;
   private ReferenceFrame recordFrame;
   String fileName = "1_New_World_WORLD_FRAME";
   TimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private static File file;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private FullRobotModel fullRobotModel;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, false, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());

      file = new File(ScriptEngineSettings.scriptSavingDirectory + fileName + ScriptEngineSettings.extension);
      System.out.println(file.getAbsolutePath());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test(timeout = 300000)
   public void testFingerStatePacketScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      int robotSidePick = new Random().nextInt(RobotSide.values().length);
      RobotSide robotside = RobotSide.values()[robotSidePick];

      FingerStatePacket fingerStatePacket = new FingerStatePacket(robotside, FingerState.CLOSE);

      // Record Script
      currentScriptFrame = worldFrame;
      recordFrame = currentScriptFrame;

      CommandRecorder commandRecorder = new CommandRecorder(timestampProvider);
      commandRecorder.startRecording(fileName, recordFrame);

      commandRecorder.recordObject(fingerStatePacket);

      // Stop Recording - Save File
      commandRecorder.stopRecording();

      ScriptBehavior scriptBehavior = testScriptBehavior(5.0);

      assertTrue(scriptBehavior.isDone());
   }

   private ScriptBehavior testScriptBehavior(double trajectoryTime) throws SimulationExceededMaximumTimeException, FileNotFoundException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      BooleanYoVariable doubleSupport = new BooleanYoVariable("doubleSupport", robot.getRobotsYoVariableRegistry());
      doubleSupport.set(true);
      
      final ScriptBehavior scriptBehavior = new ScriptBehavior(communicationBridge, fullRobotModel, yoTime, doubleSupport);
      communicationBridge.attachGlobalListenerToController(scriptBehavior.getControllerGlobalPacketConsumer());

      InputStream inputStream = new FileInputStream(new File(file.getAbsolutePath()));
      RigidBodyTransform scriptTransformToWorld = recordFrame.getTransformToDesiredFrame(worldFrame);

      scriptBehavior.initialize();
      scriptBehavior.importChildInputPackets(inputStream.toString(), inputStream, scriptTransformToWorld);

      success &= executeBehavior(scriptBehavior, trajectoryTime);

      assertTrue(success);

      return scriptBehavior;
   }

   private boolean executeBehavior(final BehaviorInterface behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      if (DEBUG)
      {
         System.out.println("\n");
         SysoutTool.println("starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
      }

      Thread behaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;

               while (simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }

                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;

                  behavior.doControl();
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      if (DEBUG)
      {
         SysoutTool.println("done with behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
      }

      return ret;
   }

   public void recordScript(Object objectToBeRecorded)
   {
      currentScriptFrame = worldFrame;

      recordFrame = currentScriptFrame;

      // System.out.println("WORLD - Record TRANS: " + recordTransform);

      // Begin recording process
      commandRecorder(objectToBeRecorded);
   }

   public void commandRecorder(Object object)
   {
      CommandRecorder commandRecorder = new CommandRecorder(timestampProvider);
      commandRecorder.startRecording(fileName, recordFrame);

      commandRecorder.recordObject(object);

      // Stop Recording - Save File
      commandRecorder.stopRecording();

      // Visualize Script
   }

   public ArrayList<ScriptObject> getScriptObjects()
   {
      ScriptFileLoader loader;
      try
      {
         loader = new ScriptFileLoader(file.getAbsolutePath());
         ArrayList<ScriptObject> scriptObjects = loader.readIntoList();
         loader.close();

         return scriptObjects;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

}
