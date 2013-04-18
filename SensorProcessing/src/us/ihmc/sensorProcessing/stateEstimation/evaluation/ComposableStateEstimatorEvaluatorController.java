package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;

public class ComposableStateEstimatorEvaluatorController implements RobotController
{
   private final Vector3d gravitationalAcceleration;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControlFlowGraph controlFlowGraph;
   private final Runnable sensorReader;
   
   private final ComposableStateEstimatorEvaluatorErrorCalculator composableStateEstimatorEvaluatorErrorCalculator;

   public ComposableStateEstimatorEvaluatorController(Runnable sensorReader,
         ControlFlowGraph controlFlowGraph, OrientationEstimator orientationEstimator,
           Robot robot, Joint estimationJoint, double controlDT)
   {
      this.sensorReader = sensorReader;
      
      this.gravitationalAcceleration = new Vector3d();
      robot.getGravity(gravitationalAcceleration);

      this.controlFlowGraph = controlFlowGraph;
      this.composableStateEstimatorEvaluatorErrorCalculator = new ComposableStateEstimatorEvaluatorErrorCalculator(robot, estimationJoint, orientationEstimator, registry);
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
      sensorReader.run();
      
      controlFlowGraph.startComputation();
      controlFlowGraph.waitUntilComputationIsDone();

      composableStateEstimatorEvaluatorErrorCalculator.computeErrors();
   }
}
