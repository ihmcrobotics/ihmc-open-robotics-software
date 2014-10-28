package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.communication.subscribers.ExternalPelvisPoseSubscriberInterface;
import us.ihmc.darpaRoboticsChallenge.controllers.PIDLidarTorqueController;
import us.ihmc.darpaRoboticsChallenge.controllers.concurrent.ThreadDataSynchronizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriterWithTorqueOffsets;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCSimulationOutputWriter;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.robotDataCommunication.VisualizerUtils;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.net.TimestampProvider;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.UnreasonableAccelerationException;
import com.yobotics.simulationconstructionset.robotController.AbstractThreadedRobotController;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotController;
import com.yobotics.simulationconstructionset.robotController.SingleThreadedRobotController;

public class DRCSimulationFactory
{
   public static boolean RUN_MULTI_THREADED = true;

   public static final boolean DO_SLOW_INTEGRATION_FOR_TORQUE_OFFSET = false;
   
   private static final double gravity = -9.81;

   private final SimulationConstructionSet scs;
   private final SDFRobot simulatedRobot;

   private DRCEstimatorThread drcEstimatorThread;
   private DRCControllerThread drcControllerThread;
   private AbstractThreadedRobotController multiThreadedRobotController;

   private final SimulatedDRCRobotTimeProvider simulatedDRCRobotTimeProvider;

   public DRCSimulationFactory(DRCRobotModel drcRobotModel, MomentumBasedControllerFactory controllerFactory, Graphics3DObject environmentGraphics,
         DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCSCSInitialSetup scsInitialSetup, DRCGuiInitialSetup guiInitialSetup,
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
         VisualizerUtils.createOverheadPlotter(scs, guiInitialSetup.isShowOverheadView(), drcControllerThread.getDynamicGraphicObjectsListRegistry(),
               drcEstimatorThread.getDynamicGraphicObjectsListRegistry());
      }

      guiInitialSetup.initializeGUI(scs, simulatedRobot, drcRobotModel);

      if (environmentGraphics != null)
      {
         scs.addStaticLinkGraphics(environmentGraphics);
      }

      scsInitialSetup.initializeRobot(simulatedRobot, null);
      robotInitialSetup.initializeRobot(simulatedRobot, drcRobotModel.getJointMap());
      simulatedRobot.update();

      setupJointDamping(simulatedRobot, drcRobotModel);
   }

   public FullRobotModelCorruptor getFullRobotModelCorruptor()
   {
      return drcControllerThread.getFullRobotModelCorruptor();
   }
   
   private void createRobotController(DRCRobotModel drcRobotModel, MomentumBasedControllerFactory controllerFactory, GlobalDataProducer globalDataProducer,
         SDFRobot simulatedRobot, SimulationConstructionSet scs, DRCSCSInitialSetup scsInitialSetup,
         DRCRobotInitialSetup<SDFRobot> robotInitialSetup)
   {
      StateEstimatorParameters stateEstimatorParameters = drcRobotModel.getStateEstimatorParameters();
      SensorNoiseParameters sensorNoiseParameters = scsInitialSetup.getSimulatedSensorNoiseParameters();

      SensorReaderFactory sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(simulatedRobot, sensorNoiseParameters,
            stateEstimatorParameters.getSensorFilterParameters(), drcRobotModel.getSensorInformation().getIMUSensorsToUse());

      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(drcRobotModel);
      DRCOutputWriter drcOutputWriter = new DRCSimulationOutputWriter(simulatedRobot);

      if (DO_SLOW_INTEGRATION_FOR_TORQUE_OFFSET)
      {
         DRCOutputWriterWithTorqueOffsets outputWriterWithTorqueOffsets = new DRCOutputWriterWithTorqueOffsets(drcOutputWriter, drcRobotModel.getControllerDT(), true);
         drcOutputWriter = outputWriterWithTorqueOffsets;
      }
      
      
      drcEstimatorThread = new DRCEstimatorThread(drcRobotModel, sensorReaderFactory, threadDataSynchronizer, globalDataProducer,
            null, gravity);

      drcControllerThread = new DRCControllerThread(drcRobotModel, controllerFactory, threadDataSynchronizer, drcOutputWriter,
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
      MultiThreadedRobotControlElement simulatedHandController = drcRobotModel.createSimulatedHandController(simulatedRobot, threadDataSynchronizer, globalDataProducer);
      if (simulatedHandController != null)
      {
         multiThreadedRobotController.addController(simulatedHandController, controllerTicksPerSimulationTick, true);
      }


      if (scsInitialSetup.getInitializeEstimatorToActual())
      {
         System.err.println("Warning! Initializing Estimator to Actual!");
         initializeEstimatorToActual(drcEstimatorThread, robotInitialSetup, simulatedRobot, drcRobotModel.getJointMap());
      }

      simulatedRobot.setController(multiThreadedRobotController);
      DRCRobotSensorInformation sensorInformation = drcRobotModel.getSensorInformation();
      DRCRobotLidarParameters lidarParams = sensorInformation.getLidarParameters(0);
      if(lidarParams != null && lidarParams.getLidarSpindleJointName() != null)
      {
         PIDLidarTorqueController lidarControllerInterface = new PIDLidarTorqueController(simulatedRobot,
               lidarParams.getLidarSpindleJointName(), lidarParams.getLidarSpindleVelocity(), drcRobotModel.getSimulateDT());
         simulatedRobot.setController(lidarControllerInterface);
      }
      simulatedRobot.setController(simulatedDRCRobotTimeProvider);
   }

   private static void setupJointDamping(SDFRobot simulatedRobot, DRCRobotModel robotModel)
   {
      robotModel.setJointDamping(simulatedRobot);
   }

   private static void initializeEstimatorToActual(DRCEstimatorThread drcStateEstimator, DRCRobotInitialSetup<SDFRobot> robotInitialSetup,
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

      RigidBodyTransform estimationLinkTransform3D = estimationJoint.getJointTransform3D();
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

   public void start()
   {
      Thread simThread = new Thread(scs, "SCS simulation thread");
      simThread.start();
   }
   
   public void simulate()
   {
      scs.simulate();
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
   
   public void setExternelPelvisCorrectorSubscriber(ExternalPelvisPoseSubscriberInterface externelPelvisCorrectorSubscriber)
   {
      drcEstimatorThread.setExternelPelvisCorrectorSubscriber(externelPelvisCorrectorSubscriber);
   }
}
