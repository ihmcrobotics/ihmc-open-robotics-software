package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.PIDLidarTorqueController;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.ScsInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCSimulationOutputWriter;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCSimulatedSensorNoiseParameters;
import us.ihmc.darpaRoboticsChallenge.stateEstimation.DRCStateEstimatorInterface;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.robotController.AbstractThreadedRobotController;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotController;
import com.yobotics.simulationconstructionset.robotController.SingleThreadedRobotController;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class DRCSimulationFactory
{
   private static final boolean RUN_MULTI_THREADED = true;

   private static final double gravity = -9.81;

   private final SimulationConstructionSet scs;
   private final SDFRobot simulatedRobot;

   private DRCEstimatorThread drcEstimatorThread;
   private DRCControllerThread drcControllerThread;
   private AbstractThreadedRobotController multiThreadedRobotController;

   private final SimulatedDRCRobotTimeProvider simulatedDRCRobotTimeProvider;

   public DRCSimulationFactory(DRCRobotModel drcRobotModel, ControllerFactory controllerFactory, TerrainObject environmentTerrain,
         DRCRobotInitialSetup<SDFRobot> robotInitialSetup, ScsInitialSetup scsInitialSetup, DRCGuiInitialSetup guiInitialSetup,
         GlobalDataProducer globalDataProducer)
   {
      simulatedDRCRobotTimeProvider = new SimulatedDRCRobotTimeProvider(drcRobotModel.getSimulateDT());
      simulatedRobot = drcRobotModel.createSdfRobot(false);

      scs = new SimulationConstructionSet(simulatedRobot, guiInitialSetup.getGraphics3DAdapter(), scsInitialSetup.getSimulationDataBufferSize());
      scs.setDT(drcRobotModel.getSimulateDT(), 1);

      createRobotController(drcRobotModel, controllerFactory, globalDataProducer, simulatedRobot, scs, scsInitialSetup, robotInitialSetup);

      simulatedRobot.setDynamicIntegrationMethod(scsInitialSetup.getDynamicIntegrationMethod());

      scsInitialSetup.initializeSimulation(scs);

      if (guiInitialSetup.isGuiShown())
      {
         VisualizerUtils.createOverheadPlotter(scs, DRCLocalConfigParameters.SHOW_OVERHEAD_VIEW, drcControllerThread.getDynamicGraphicObjectsListRegistry(),
               drcEstimatorThread.getDynamicGraphicObjectsListRegistry());
      }

      guiInitialSetup.initializeGUI(scs, simulatedRobot, drcRobotModel);

      if (environmentTerrain != null)
      {
         scs.addStaticLinkGraphics(environmentTerrain.getLinkGraphics());
      }

      scsInitialSetup.initializeRobot(simulatedRobot, null);
      robotInitialSetup.initializeRobot(simulatedRobot, drcRobotModel.getJointMap());
      simulatedRobot.update();

      setupJointDamping(simulatedRobot, drcRobotModel);
   }

   private void createRobotController(DRCRobotModel drcRobotModel, ControllerFactory controllerFactory, GlobalDataProducer globalDataProducer,
         SDFRobot simulatedRobot, SimulationConstructionSet scs, ScsInitialSetup scsInitialSetup,
         DRCRobotInitialSetup<SDFRobot> robotInitialSetup)
   {
      SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise();
      StateEstimatorParameters stateEstimatorParameters = drcRobotModel.getStateEstimatorParameters();

      LidarControllerInterface lidarControllerInterface = new PIDLidarTorqueController(DRCConfigParameters.LIDAR_SPINDLE_VELOCITY,
            drcRobotModel.getSimulateDT());
      SensorReaderFactory sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(simulatedRobot, sensorNoiseParameters,
            stateEstimatorParameters.getSensorFilterParameters(), drcRobotModel.getSensorInformation().getIMUSensorsToUse());

      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(drcRobotModel);
      DRCOutputWriter drcOutputWriter = new DRCSimulationOutputWriter(simulatedRobot);
      if (DRCLocalConfigParameters.INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
      {
         drcOutputWriter = drcRobotModel.getOutputWriterWithAccelerationIntegration(drcOutputWriter, false);
      }

      drcEstimatorThread = new DRCEstimatorThread(drcRobotModel, sensorReaderFactory, threadDataSynchronizer, globalDataProducer,
            null, drcRobotModel.getEstimatorDT(), gravity);

      drcControllerThread = new DRCControllerThread(drcRobotModel, controllerFactory, lidarControllerInterface, threadDataSynchronizer, drcOutputWriter,
            globalDataProducer, null, gravity);

      if (RUN_MULTI_THREADED)
      {
         multiThreadedRobotController = new MultiThreadedRobotController("DRCSimulation", simulatedRobot.getYoTime(), scs);
      }
      else
      {
         System.err.println("[WARNING] Running simulation in single threaded mode");
         multiThreadedRobotController = new SingleThreadedRobotController("DRCSimulation", simulatedRobot.getYoTime(), scs);
      }
      int estimatorTicksPerSimulationTick = (int) Math.round(drcRobotModel.getEstimatorDT() / drcRobotModel.getSimulateDT());
      int controllerTicksPerSimulationTick = (int) Math.round(drcRobotModel.getControllerDT() / drcRobotModel.getSimulateDT());
      multiThreadedRobotController.addController(drcEstimatorThread, estimatorTicksPerSimulationTick, false);
      multiThreadedRobotController.addController(drcControllerThread, controllerTicksPerSimulationTick, true);

      if (scsInitialSetup.getInitializeEstimatorToActual())
      {
         System.err.println("Warning! Initializing Estimator to Actual!");
         DRCStateEstimatorInterface drcStateEstimator = drcEstimatorThread.getDRCStateEstimator();
         initializeEstimatorToActual(drcStateEstimator, robotInitialSetup, simulatedRobot, drcRobotModel.getJointMap());
      }

      simulatedRobot.setController(multiThreadedRobotController);
      simulatedRobot.setController(lidarControllerInterface, estimatorTicksPerSimulationTick);
      simulatedRobot.setController(simulatedDRCRobotTimeProvider);
   }

   private static void setupJointDamping(SDFRobot simulatedRobot, DRCRobotModel robotModel)
   {
      robotModel.setJointDamping(simulatedRobot);
   }

   private static void initializeEstimatorToActual(DRCStateEstimatorInterface drcStateEstimator, DRCRobotInitialSetup<SDFRobot> robotInitialSetup,
         SDFRobot simulatedRobot, DRCRobotJointMap jointMap)
   {
      // The following is to get the initial CoM position from the robot. 
      // It is cheating for now, and we need to move to where the 
      // robot itself determines coordinates, and the sensors are all
      // in the robot-determined world coordinates..
      Point3d initialCoMPosition = new Point3d();
      robotInitialSetup.initializeRobot(simulatedRobot, jointMap);
      updateRobot(simulatedRobot);

      simulatedRobot.computeCenterOfMass(initialCoMPosition);

      Joint estimationJoint = simulatedRobot.getPelvisJoint();

      Transform3D estimationLinkTransform3D = estimationJoint.getJointTransform3D();
      Quat4d initialEstimationLinkOrientation = new Quat4d();
      estimationLinkTransform3D.get(initialEstimationLinkOrientation);

      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimatorToActual(initialCoMPosition, initialEstimationLinkOrientation);
   }

   private static void updateRobot(Robot robot)
   {
      try
      {
         robot.update();
         robot.doDynamicsButDoNotIntegrate();
         robot.update();
      }
      catch (UnreasonableAccelerationException e)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public void start(AutomaticSimulationRunner automaticSimulationRunner)
   {
      Thread simThread = new Thread(scs, "SCS simulation thread");
      simThread.start();

      if (automaticSimulationRunner != null)
      {
         try
         {
            automaticSimulationRunner.automaticallyRunSimulation(scs);
         }
         catch (SimulationExceededMaximumTimeException e)
         {
            throw new RuntimeException("Simulation Exceeded maximum runtime!");
         }
      }
   }

   public void dispose()
   {
      multiThreadedRobotController.stop();
      drcEstimatorThread = null;
      drcControllerThread = null;
      multiThreadedRobotController = null;
   }

   public SDFRobot getRobot()
   {
      return simulatedRobot;
   }

   public TimestampProvider getTimeStampProvider()
   {
      return simulatedDRCRobotTimeProvider;
   }
}
