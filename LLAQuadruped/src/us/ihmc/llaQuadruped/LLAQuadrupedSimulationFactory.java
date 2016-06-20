package us.ihmc.llaQuadruped;

import java.io.IOException;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.llaQuadruped.simulation.LLAQuadrupedGroundContactParameters;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.factories.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class LLAQuadrupedSimulationFactory extends QuadrupedSimulationFactory
{
   private static final QuadrupedControlMode CONTROL_MODE = QuadrupedControlMode.FORCE;
   private final QuadrupedGroundContactModelType groundContactModelType = QuadrupedGroundContactModelType.FLAT;
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;
   
   @Override
   public SimulationConstructionSet createSimulation() throws IOException
   {      
      LLAQuadrupedModelFactory modelFactory = new LLAQuadrupedModelFactory();
      SDFRobot sdfRobot = modelFactory.createSdfRobot();
      SDFFullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      LLAQuadrupedPhysicalProperties physicalProperties = new LLAQuadrupedPhysicalProperties();
      
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new SDFPerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);
      
      SensorReader sensorReader = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot, fullRobotModel, referenceFrames);
      
      setControlDT(SIMULATION_DT);
      setGravity(SIMULATION_GRAVITY);
      setRecordFrequency(RECORD_FREQUENCY);
      setGroundContactModelType(groundContactModelType);
      setGroundContactParameters(new LLAQuadrupedGroundContactParameters());
      setHeadController(null);
      setModelFactory(modelFactory);
      setSCSParameters(new SimulationConstructionSetParameters());
      setSDFRobot(sdfRobot);
      setStateEstimator(null);
      setSensorReader(sensorReader);
      setOutputWriter(outputWriter);
      setShowPlotter(SHOW_PLOTTER);
      setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      setStandPrepParameters(new LLAQuadrupedStandPrepParameters());
      setFullRobotModel(fullRobotModel);
      setKryoNetClassList(new LLAQuadrupedNetClassList());
      setPhysicalProperties(physicalProperties);
      setReferenceFrames(referenceFrames);
      setControlMode(CONTROL_MODE);
      
      return super.createSimulation();
   }

   public static void main(String[] commandLineArguments)
   {
      try
      {
         LLAQuadrupedSimulationFactory simulationFactory = new LLAQuadrupedSimulationFactory();
         SimulationConstructionSet scs = simulationFactory.createSimulation();
         scs.startOnAThread();
         scs.simulate();
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
      }
   }
}
