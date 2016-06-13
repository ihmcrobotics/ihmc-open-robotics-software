package us.ihmc.llaQuadruped;

import java.io.IOException;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.llaQuadruped.simulation.LLAQuadrupedGroundContactParameters;
import us.ihmc.quadrupedRobotics.communication.QuadrupedGlobalDataProducer;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerManager;
import us.ihmc.quadrupedRobotics.controller.forceDevelopment.QuadrupedForceDevelopmentControllerManager;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.AlternatingSlopesGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RotatablePlaneTerrainProfile;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class LLAQuadrupedSimulationFactory
{
   private static final boolean USE_FORCE_DEVELOPMENT = false;
   
   private static final double SIMULATION_DT = 0.000125;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final boolean SHOW_OVERHEAD_VIEW = false;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   
   private final GroundContactModelType groundContactModelType = GroundContactModelType.FLAT;

   private enum GroundContactModelType
   {
      FLAT, ROTATABLE, ROLLING_HILLS, SLOPES
   }
   
   // First stage
   private LLAQuadrupedModelFactory modelFactory;
   private SDFRobot sdfRobot;
   private SDFFullQuadrupedRobotModel fullRobotModel;
   private YoVariableRegistry robotsYoVariableRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private LLAQuadrupedPhysicalProperties physicalProperties;
   private RobotController headController;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private DRCPoseCommunicator poseCommunicator;
   private LLAQuadrupedStandPrepParameters standPrepParameters;
   private LLAQuadrupedGroundContactParameters groundContactParameters;
   private Point3d initialCoMPosition;
   private SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
   private LLAQuadrupedNetClassList netClassList;
   
   // Second stage
   private PacketCommunicator packetCommunicator;
   private QuadrupedReferenceFrames referenceFrames;
   private SDFPerfectSimulatedOutputWriter sdfPerfectSimulatedOutputWriter;
   private GroundProfile3D groundProfile3D;
   
   // Third stage
   private GlobalDataProducer globalDataProducer;
   private QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private SDFQuadrupedPerfectSimulatedSensor sdfQuadrupedPerfectSimulatedSensor;
   private LinearGroundContactModel groundContactModel;
   
   // Fourth stage
   private QuadrantDependentList<FootSwitchInterface> footSwitches;
   
   // Fifth stage
   private QuadrupedRuntimeEnvironment runtimeEnvironment;
   
   // Sixth stage
   private QuadrupedControllerManager controllerManager;
   
   // Seventh stage
   private QuadrupedSimulationController simulationController;
   
   // Ninth stage
   private SimulationConstructionSet scs;


   private void createFirstStage()
   {
      modelFactory = new LLAQuadrupedModelFactory();
      sdfRobot = modelFactory.createSdfRobot();
      fullRobotModel = modelFactory.createFullRobotModel();
      robotsYoVariableRegistry = sdfRobot.getRobotsYoVariableRegistry();
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      yoGraphicsListRegistryForDetachedOverhead = new YoGraphicsListRegistry();
      netClassList = new LLAQuadrupedNetClassList();
      headController = null;
      stateEstimator = null;
      poseCommunicator = null;
      physicalProperties = new LLAQuadrupedPhysicalProperties();
      standPrepParameters = new LLAQuadrupedStandPrepParameters();
      initialCoMPosition = new Point3d();
      groundContactParameters = new LLAQuadrupedGroundContactParameters();
   }
   
   private void createSecondStage()
   {
      packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, netClassList);
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      sdfPerfectSimulatedOutputWriter = new SDFPerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);
      
      switch (groundContactModelType)
      {
      case FLAT:
         groundProfile3D = new FlatGroundProfile(0.0);
         break;
      case ROLLING_HILLS:
      groundProfile3D =  new RollingGroundProfile(0.025, 1.0, 0.0, -20.0, 20.0, -20.0, 20.0);
         break;
      case ROTATABLE:
         groundProfile3D = new RotatablePlaneTerrainProfile(new Point3d(), sdfRobot, yoGraphicsListRegistry, SIMULATION_DT);
         break;
      case SLOPES:
         double xMin = -5.0, xMax = 40.0;
         double yMin = -5.0, yMax =  5.0;
         double[][] xSlopePairs = new double[][]
         {
            {1.0, 0.0}, {3.0, 0.1}
         };
         groundProfile3D = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);
         break;
      default:
         groundProfile3D = null;
         break;
      }
   }
   
   private void createThirdStage()
   {
      globalDataProducer = new QuadrupedGlobalDataProducer(packetCommunicator);
      contactableFeet = QuadrupedStateEstimatorFactory.createFootContactableBodies(fullRobotModel, referenceFrames, physicalProperties);
      sdfQuadrupedPerfectSimulatedSensor = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot, fullRobotModel, referenceFrames);
      
      groundContactModel = new LinearGroundContactModel(sdfRobot, robotsYoVariableRegistry);
      groundContactModel.setZStiffness(groundContactParameters.getZStiffness());
      groundContactModel.setZDamping(groundContactParameters.getZDamping());
      groundContactModel.setXYStiffness(groundContactParameters.getXYStiffness());
      groundContactModel.setXYDamping(groundContactParameters.getXYDamping());
      groundContactModel.setGroundProfile3D(groundProfile3D);
   }
   
   private void createFourthStage()
   {
      footSwitches = QuadrupedStateEstimatorFactory.createFootSwitches(contactableFeet, SIMULATION_GRAVITY, fullRobotModel, robotsYoVariableRegistry);
   }
   
   private void createFifthStage()
   {
      runtimeEnvironment = new QuadrupedRuntimeEnvironment(SIMULATION_DT, sdfRobot.getYoTime(), fullRobotModel, robotsYoVariableRegistry,
                                                           yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead, globalDataProducer, footSwitches);
   }
   
   private void createSixthStage() throws IOException
   {
      if (USE_FORCE_DEVELOPMENT)
      {
         controllerManager = new QuadrupedForceDevelopmentControllerManager(runtimeEnvironment, physicalProperties);
      }
      else
      {
         controllerManager = new QuadrupedForceControllerManager(runtimeEnvironment, physicalProperties);
      }
   }
   
   private void createSeventhStage()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot, sdfQuadrupedPerfectSimulatedSensor, sdfPerfectSimulatedOutputWriter, controllerManager, stateEstimator,
                                        poseCommunicator, headController);
   }
   
   private void createEighthStage()
   {
      sdfRobot.setController(simulationController);
      sdfRobot.setPositionInWorld(new Vector3d(0.0, 0.0, standPrepParameters.getInitialHeight()));
      try
      {
         sdfRobot.update();
         sdfRobot.doDynamicsButDoNotIntegrate();
         sdfRobot.update();
      }
      catch (UnreasonableAccelerationException unreasonableAccelerationException)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }
      double totalMass = sdfRobot.computeCenterOfMass(initialCoMPosition);
      sdfRobot.setGravity(SIMULATION_GRAVITY);
      sdfRobot.setGroundContactModel(groundContactModel);
      System.out.println("Total mass: " + totalMass);
   }
   
   private void createNinthStage()
   {
      scs = new SimulationConstructionSet(sdfRobot, scsParameters);
      if (groundContactModelType == GroundContactModelType.ROTATABLE)
      {
         scs.setGroundVisible(false);
      }
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotter(scs, SHOW_OVERHEAD_VIEW, "centerOfMass", yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotterInSeparateWindow(scs, SHOW_OVERHEAD_VIEW, "centerOfMass", yoGraphicsListRegistryForDetachedOverhead);
      scs.setDT(SIMULATION_DT, RECORD_FREQUENCY);
      if (scs.getSimulationConstructionSetParameters().getCreateGUI())
      {
         scs.setCameraTrackingVars("q_x", "q_y", "q_z");
         scs.setCameraDollyVars("q_x", "q_y", "q_z");
         scs.setCameraTracking(true, true, true, true);
         scs.setCameraDolly(true, true, true, false);
         scs.setCameraDollyOffsets(4.0, 4.0, 1.0);
         scs.getStandardSimulationGUI().selectPanel("Plotter");
      }
   }
   
   public void createSimulation() throws IOException
   {
      createFirstStage();
      createSecondStage();
      createThirdStage();
      createFourthStage();
      createFifthStage();
      createSixthStage();
      createSeventhStage();
      createEighthStage();
      createNinthStage();
   }
   
   public void start()
   {
      scs.startOnAThread();
      scs.simulate();
   }

   public static void main(String[] commandLineArguments)
   {
      try
      {
         LLAQuadrupedSimulationFactory simulationFactory = new LLAQuadrupedSimulationFactory();
         simulationFactory.createSimulation();
         simulationFactory.start();
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
      }
   }
}
