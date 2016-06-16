package us.ihmc.llaQuadruped;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.llaQuadruped.simulation.LLAQuadrupedGroundContactParameters;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.factories.QuadrupedControllerManagerFactory;
import us.ihmc.quadrupedRobotics.factories.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.params.ParameterRegistry;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class LLAQuadrupedSimulationFactory
{
   private static final boolean USE_FORCE_DEVELOPMENT = false;
   
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   
   private final QuadrupedGroundContactModelType groundContactModelType = QuadrupedGroundContactModelType.FLAT;
   
   // Factories
   QuadrupedControllerManagerFactory controllerManagerFactory = new QuadrupedControllerManagerFactory();
   QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();
   
   // First stage
   private LLAQuadrupedModelFactory modelFactory;
   private SDFRobot sdfRobot;
   private SDFFullQuadrupedRobotModel fullRobotModel;
   private YoVariableRegistry robotYoVariableRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private LLAQuadrupedPhysicalProperties physicalProperties;
   private RobotController headController;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private LLAQuadrupedStandPrepParameters standPrepParameters;
   private LLAQuadrupedGroundContactParameters groundContactParameters;
   private SimulationConstructionSetParameters scsParameters;
   private LLAQuadrupedNetClassList netClassList;
   
   // Second stage
   private QuadrupedReferenceFrames referenceFrames;
   private SDFPerfectSimulatedOutputWriter sdfPerfectSimulatedOutputWriter;
   
   // Third stage
   private SDFQuadrupedPerfectSimulatedSensor sdfQuadrupedPerfectSimulatedSensor;
   
   // Sixth stage
   private QuadrupedControllerManager controllerManager;
   
   // Ninth stage
   private SimulationConstructionSet scs;


   private void createSimulationParameters() throws IOException
   {
      modelFactory = new LLAQuadrupedModelFactory();
      sdfRobot = modelFactory.createSdfRobot();
      fullRobotModel = modelFactory.createFullRobotModel();
      robotYoVariableRegistry = sdfRobot.getRobotsYoVariableRegistry();
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      yoGraphicsListRegistryForDetachedOverhead = new YoGraphicsListRegistry();
      netClassList = new LLAQuadrupedNetClassList();
      headController = null;
      stateEstimator = null;
      physicalProperties = new LLAQuadrupedPhysicalProperties();
      standPrepParameters = new LLAQuadrupedStandPrepParameters();
      groundContactParameters = new LLAQuadrupedGroundContactParameters();
      scsParameters = new SimulationConstructionSetParameters();
      ParameterRegistry.getInstance().loadFromResources("parameters/simulation.param");
      
      referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      sdfPerfectSimulatedOutputWriter = new SDFPerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);
      
      sdfQuadrupedPerfectSimulatedSensor = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot, fullRobotModel, referenceFrames);
   }
   
   private void createControllerManager() throws IOException
   {
      controllerManagerFactory.setControlDT(SIMULATION_DT);
      controllerManagerFactory.setGravity(SIMULATION_GRAVITY);
      controllerManagerFactory.setFullRobotModel(fullRobotModel);
      controllerManagerFactory.setKryoNetClassList(netClassList);
      controllerManagerFactory.setPhysicalProperties(physicalProperties);
      controllerManagerFactory.setReferenceFrames(referenceFrames);
      controllerManagerFactory.setRobotYoVariableRegistry(robotYoVariableRegistry);
      controllerManagerFactory.setTimestampYoVariable(sdfRobot.getYoTime());
      controllerManagerFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
      controllerManagerFactory.setYoGraphicsListRegistryForDetachedOverhead(yoGraphicsListRegistryForDetachedOverhead);

      if (USE_FORCE_DEVELOPMENT)
      {
         controllerManager = controllerManagerFactory.createForceDevelopmentControllerManager();
      }
      else
      {
         controllerManager = controllerManagerFactory.createForceControllerManager();
      }
   }
   
   private void createSCS()
   {
      simulationFactory.setControlDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setControllerManager(controllerManager);
      simulationFactory.setGroundContactModelType(groundContactModelType);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setHeadController(headController);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSCSParameters(scsParameters);
      simulationFactory.setSDFRobot(sdfRobot);
      simulationFactory.setStateEstimator(stateEstimator);
      simulationFactory.setSensorReader(sdfQuadrupedPerfectSimulatedSensor);
      simulationFactory.setOutputWriter(sdfPerfectSimulatedOutputWriter);
      simulationFactory.setShowPlotter(true);
      simulationFactory.setUseTrackAndDolly(false);
      simulationFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
      simulationFactory.setYoGraphicsListRegistryForDetachedOverhead(yoGraphicsListRegistryForDetachedOverhead);
      simulationFactory.setStandPrepParameters(standPrepParameters);
      
      scs = simulationFactory.createSimulation();
   }
   
   public void createSimulation() throws IOException
   {
      createSimulationParameters();
      createControllerManager();
      createSCS();
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
