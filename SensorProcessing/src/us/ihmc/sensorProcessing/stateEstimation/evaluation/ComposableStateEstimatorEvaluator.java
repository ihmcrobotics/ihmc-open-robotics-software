package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.JointStateFullRobotModelUpdater;
import us.ihmc.sensorProcessing.stateEstimation.SimulatedSensorController;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

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

      InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(robot);
      StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(generator, robot, robot.getIMUMounts(),
                                                                       robot.getVelocityPoints());

      SimulatedSensorController simulatedSensorController = new SimulatedSensorController(perfectFullRobotModel, generator, robot, controlDT);
      SensorMap sensorMap = simulatedSensorController.getSensorMap();

//      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), perfectFullRobotModel.getElevator());
//      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(perfectFullRobotModel.getElevator(), twistCalculator,
//                                                                       0.0, false);


      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
         new DesiredCoMAccelerationsFromRobotStealerController(perfectFullRobotModel, generator, robot, controlDT);

      StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel = simulatedSensorController.getStateEstimatorEvaluatorFullRobotModel();


      ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController = new ComposableStateEstimatorEvaluatorController(robot,
                                                                                                   estimatedFullRobotModel,
                                                                                                   controlDT, sensorMap,
                                                                                                   desiredCoMAccelerationsFromRobotStealerController);
      robot.setController(simulatedSensorController, simTicksPerControlDT);
      robot.setController(desiredCoMAccelerationsFromRobotStealerController, simTicksPerControlDT);
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
