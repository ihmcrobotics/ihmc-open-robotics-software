package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMapFromRobotFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationDataSource;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAndAngularAccelerationOutputPortsHolder;
import us.ihmc.sensorProcessing.stateEstimation.OrientationEstimatorWithPorts;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.Joint;
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
      ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();
      robot.getIMUMounts(imuMounts);

      SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap = generator.getSCSToInverseDynamicsJointMap();

      SensorNoiseParameters simulatedSensorNoiseParameters = SensorNoiseParametersForEvaluator.createSensorNoiseParameters();
      SensorMapFromRobotFactory sensorMapFromRobotFactory = new SensorMapFromRobotFactory(scsToInverseDynamicsJointMap, robot, simulatedSensorNoiseParameters,
                                                               controlDT, imuMounts, robot.getVelocityPoints(), registry);
      SensorMap sensorMap = sensorMapFromRobotFactory.getSensorMap();

      Joint estimationJoint = robot.getRootJoint();
      robot.update();
      FullInverseDynamicsStructure inverseDynamicsStructure = generator.getInverseDynamicsStructure();

      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();

      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController =
         new DesiredCoMAccelerationsFromRobotStealerController(estimationFrame, simulatedSensorNoiseParameters, generator, estimationJoint, controlDT);

      Vector3d gravitationalAcceleration = new Vector3d();
      robot.getGravity(gravitationalAcceleration);

      // The following few lines are what you need to do to get the state estimator working with a robot.
      // You also need to either add the controlFlowGraph to another one, or make sure to run it's startComputation method at the right time:
      SensorNoiseParameters sensorNoiseParametersForEstimator = SensorNoiseParametersForEvaluator.createSensorNoiseParameters();
      
      SensorAndEstimatorAssembler sensorAndEstimatorAssembler = new SensorAndEstimatorAssembler(sensorNoiseParametersForEstimator, gravitationalAcceleration,
                                                                   inverseDynamicsStructure, controlDT, sensorMap,
                                                                   registry);

      ControlFlowGraph controlFlowGraph = sensorAndEstimatorAssembler.getControlFlowGraph();
      OrientationEstimatorWithPorts orientationEstimator = sensorAndEstimatorAssembler.getOrientationEstimator();

      RigidBody estimationLink = inverseDynamicsStructure.getEstimationLink();
      DesiredCoMAndAngularAccelerationDataSource desiredCoMAndAngularAccelerationDataSource = new DesiredCoMAndAngularAccelerationDataSource(estimationLink, estimationFrame);

      desiredCoMAccelerationsFromRobotStealerController.attachDesiredCoMAndAngularAccelerationDataSource(desiredCoMAndAngularAccelerationDataSource);

      desiredCoMAndAngularAccelerationDataSource.connectDesiredAccelerationPorts(controlFlowGraph, orientationEstimator);

//      SensorAndEstimatorAssembler.connectDesiredAccelerationPorts(controlFlowGraph, orientationEstimator,
//            desiredCoMAndAngularAccelerationOutputPortsHolder);

      controlFlowGraph.initializeAfterConnections();
      

      ComposableStateEstimatorEvaluatorController composableStateEstimatorEvaluatorController =
         new ComposableStateEstimatorEvaluatorController(controlFlowGraph, orientationEstimator, robot, estimationJoint, controlDT, sensorMap);
      robot.setController(desiredCoMAccelerationsFromRobotStealerController, simTicksPerControlDT);
      robot.setController(composableStateEstimatorEvaluatorController, simTicksPerControlDT);

      if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
      {
         boolean updateRootJoints = true;
         boolean updateDesireds = false;
         generator.updateInverseDynamicsRobotModelFromRobot(updateRootJoints, updateDesireds);


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
