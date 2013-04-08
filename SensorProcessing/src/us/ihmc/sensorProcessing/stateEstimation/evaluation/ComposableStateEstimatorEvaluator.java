package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.sensorProcessing.stateEstimation.SimulatedSensorController;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableStateEstimatorEvaluator
{
   private static final boolean SHOW_GUI = true;

   private final double simDT = 1e-3;
   private final int simTicksPerControlDT = 5;
   private final double controlDT = simDT * simTicksPerControlDT;
   private final int simTicksPerRecord = simTicksPerControlDT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   public ComposableStateEstimatorEvaluator()
   {
      StateEstimatorEvaluatorRobot robot = new StateEstimatorEvaluatorRobot();
      
      SimulatedSensorController simulatedSensorController = new SimulatedSensorController(robot, controlDT);
      
      ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController = new ComposableStateEstimatorEvaluatorController(robot, controlDT, simulatedSensorController, simulatedSensorController);
      robot.setController(simulatedSensorController, simTicksPerControlDT);
      robot.setController(composableStateEstimatorEvaluatorController, simTicksPerControlDT);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, SHOW_GUI, 32000);
      scs.addYoVariableRegistry(registry);

      scs.setDT(simDT, simTicksPerRecord);
      scs.setSimulateDuration(45.0);
      scs.startOnAThread();
      scs.simulate();
   }

   public static void main(String[] args)
   {
      new ComposableStateEstimatorEvaluator();
   }

}
