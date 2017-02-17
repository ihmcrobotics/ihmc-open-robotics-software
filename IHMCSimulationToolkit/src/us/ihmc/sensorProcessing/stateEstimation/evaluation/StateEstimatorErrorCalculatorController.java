package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

public class StateEstimatorErrorCalculatorController implements RobotController
{
   private final Vector3D gravitationalAcceleration;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());   
   private final StateEstimatorErrorCalculator composableStateEstimatorEvaluatorErrorCalculator;

   public StateEstimatorErrorCalculatorController(StateEstimator orientationEstimator,
           Robot robot, Joint estimationJoint, boolean assumePerfectIMU, boolean useSimplePelvisPositionEstimator)
   {      
      this.gravitationalAcceleration = new Vector3D();
      robot.getGravity(gravitationalAcceleration);

      this.composableStateEstimatorEvaluatorErrorCalculator = new StateEstimatorErrorCalculator(robot, estimationJoint, orientationEstimator, assumePerfectIMU, useSimplePelvisPositionEstimator, registry);
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
      composableStateEstimatorEvaluatorErrorCalculator.computeErrors();
   }
}
