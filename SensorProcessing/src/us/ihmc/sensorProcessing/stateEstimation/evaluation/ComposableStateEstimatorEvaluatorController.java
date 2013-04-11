package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class ComposableStateEstimatorEvaluatorController implements RobotController
{
   private final Vector3d gravitationalAcceleration;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FullInverseDynamicsStructure inverseDynamicsStructure;

   private final ControlFlowGraph controlFlowGraph;

   private final ComposableStateEstimatorEvaluatorErrorCalculator composableStateEstimatorEvaluatorErrorCalculator;

   public ComposableStateEstimatorEvaluatorController(SensorAndEstimatorAssembler sensorAndEstimatorAssembler, StateEstimatorEvaluatorRobot robot,
           FullInverseDynamicsStructure inverseDynamicsStructure, double controlDT, SensorMap sensorMap,
           DesiredCoMAndAngularAccelerationOutputPortsHolder desiredCoMAndAngularAccelerationOutputPortsHolder)
   {
      this.gravitationalAcceleration = new Vector3d();
      robot.getGravity(gravitationalAcceleration);

      this.inverseDynamicsStructure = inverseDynamicsStructure;

      this.controlFlowGraph = sensorAndEstimatorAssembler.getControlFlowGraph();
      OrientationEstimator orientationEstimator = sensorAndEstimatorAssembler.getOrientationEstimator();

      this.composableStateEstimatorEvaluatorErrorCalculator = new ComposableStateEstimatorEvaluatorErrorCalculator(robot, orientationEstimator, registry);
   }


   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      // TODO: Not sure if this should be done here or elsewhere...
      inverseDynamicsStructure.updateInternalState();

      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      composableStateEstimatorEvaluatorErrorCalculator.computeErrors();
   }
}
