package us.ihmc.llaQuadruped;

import java.io.IOException;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.llaQuadruped.simulation.LLAQuadrupedGroundContactParameters;
import us.ihmc.quadrupedRobotics.QuadrupedTestAdministratorFactory;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.factories.QuadrupedControllerManagerFactory;
import us.ihmc.quadrupedRobotics.factories.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.params.ParameterRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class LLAQuadrupedTestFactory
{
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   
   // Parameters
   private LLAQuadrupedModelFactory modelFactory;
   private LLAQuadrupedPhysicalProperties physicalProperties;
   private LLAQuadrupedStandPrepParameters standPrepParameters;
   private LLAQuadrupedGroundContactParameters groundContactParameters;
   private LLAQuadrupedNetClassList netClassList;
   
   // Parameters getting moved soon
   private RobotController headController;
   private SimulationConstructionSetParameters scsParameters;
   private SDFRobot sdfRobot;
   private SDFFullQuadrupedRobotModel fullRobotModel;
   private YoVariableRegistry robotYoVariableRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private QuadrupedReferenceFrames referenceFrames;
   private DRCKinematicsBasedStateEstimator stateEstimator;
   private SDFPerfectSimulatedOutputWriter sdfPerfectSimulatedOutputWriter;
   private SDFQuadrupedPerfectSimulatedSensor sdfQuadrupedPerfectSimulatedSensor;
   
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

   private QuadrupedControllerManagerFactory createControllerManagerFactory() throws IOException
   {
      QuadrupedControllerManagerFactory controllerManagerFactory = new QuadrupedControllerManagerFactory();
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
      return controllerManagerFactory;
   }

   private QuadrupedSimulationFactory createSimulationFactory()
   {
      QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();
      simulationFactory.setControlDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
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
      return simulationFactory;
   }
   
   public QuadrupedTestAdministratorFactory createTestAdministratorFactory() throws IOException
   {
      createSimulationParameters();
      QuadrupedTestAdministratorFactory testAdministratorFactory = new QuadrupedTestAdministratorFactory();
      testAdministratorFactory.setConrollerManagerFactory(createControllerManagerFactory());
      testAdministratorFactory.setSimulationFactory(createSimulationFactory());
      return testAdministratorFactory;
   }
}
