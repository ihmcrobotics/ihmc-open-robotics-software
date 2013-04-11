package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimator;
import us.ihmc.sensorProcessing.stateEstimation.SimulatedSensorController;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

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

   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = true;    // false;

   public ComposableStateEstimatorEvaluator()
   {
      StateEstimatorEvaluatorRobot robot = new StateEstimatorEvaluatorRobot();

      InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(robot);
      StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(generator, robot, robot.getIMUMounts(),
                                                                       robot.getVelocityPoints());

      SimulatedSensorController simulatedSensorController = new SimulatedSensorController(perfectFullRobotModel, generator, robot, controlDT);
      SensorMap sensorMap = simulatedSensorController.getSensorMap();

      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
         new DesiredCoMAccelerationsFromRobotStealerController(perfectFullRobotModel, generator, robot, controlDT);

      StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel = simulatedSensorController.getStateEstimatorEvaluatorFullRobotModel();

      RigidBody estimationLink = estimatedFullRobotModel.getRootBody();
      RigidBody elevator = estimatedFullRobotModel.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = estimatedFullRobotModel.getRootInverseDynamicsJoint();

      FullInverseDynamicsStructure robotStuffToEstimate = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController = new ComposableStateEstimatorEvaluatorController(robot,
                                                                                                   robotStuffToEstimate, controlDT, sensorMap,
                                                                                                   desiredCoMAccelerationsFromRobotStealerController);
      robot.setController(simulatedSensorController, simTicksPerControlDT);
      robot.setController(desiredCoMAccelerationsFromRobotStealerController, simTicksPerControlDT);
      robot.setController(composableStateEstimatorEvaluatorController, simTicksPerControlDT);

      OrientationEstimator orientationEstimator = composableStateEstimatorEvaluatorController.getOrientationEstimator();

      if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
      {
         estimatedFullRobotModel.updateBasedOnRobot(true);

         Matrix3d rotationMatrix = new Matrix3d();
         robot.getRootJoint().getRotationToWorld(rotationMatrix);
         Vector3d angularVelocityInBody = robot.getRootJoint().getAngularVelocityInBody();

         FrameOrientation estimatedOrientation = orientationEstimator.getEstimatedOrientation();
         estimatedOrientation.set(rotationMatrix);
         orientationEstimator.setEstimatedOrientation(estimatedOrientation);

         FrameVector estimatedAngularVelocity = orientationEstimator.getEstimatedAngularVelocity();
         estimatedAngularVelocity.set(angularVelocityInBody);
         orientationEstimator.setEstimatedAngularVelocity(estimatedAngularVelocity);

         // TODO: This wasn't doing anything.
         // DenseMatrix64F x = orientationEstimator.getState();
         // MatrixTools.insertTuple3dIntoEJMLVector(angularVelocityInBody, x, 3);
         // orientationEstimator.setState(x, orientationEstimator.getCovariance());
      }

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
