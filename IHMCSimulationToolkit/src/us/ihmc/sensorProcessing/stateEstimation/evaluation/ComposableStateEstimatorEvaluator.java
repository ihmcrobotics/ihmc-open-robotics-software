package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.DesiredCoMAccelerationsFromRobotStealerController;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.PointMeasurementNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorWithPorts;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class ComposableStateEstimatorEvaluator
{
   private static final boolean SHOW_GUI = true;
   private final boolean assumePerfectIMU = false;
   private final boolean useSimpleComEstimator = false;
   private final double simDT = 1e-3;
   private final int simTicksPerControlDT = 5;
   private final double controlDT = simDT * simTicksPerControlDT;
   private final int simTicksPerRecord = simTicksPerControlDT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = true; // false;

   public ComposableStateEstimatorEvaluator()
   {
      StateEstimatorEvaluatorRobot robot = new StateEstimatorEvaluatorRobot();

      InverseDynamicsJointsFromSCSRobotGenerator generator = new InverseDynamicsJointsFromSCSRobotGenerator(robot);

      FullInverseDynamicsStructure inverseDynamicsStructure = generator.getInverseDynamicsStructure();

      //      SensorNoiseParameters simulatedSensorNoiseParameters = SensorNoiseParametersForEvaluator.createVeryLittleSensorNoiseParameters();
      final SensorNoiseParameters simulatedSensorNoiseParameters = SensorNoiseParametersForEvaluator.createSensorNoiseParameters();
      //      SensorNoiseParameters simulatedSensorNoiseParameters = SensorNoiseParametersForEvaluator.createZeroNoiseParameters();

      SensorProcessingConfiguration sensorProcessingConfiguration = new SensorProcessingConfiguration()
      {
         @Override
         public SensorNoiseParameters getSensorNoiseParameters()
         {
            return simulatedSensorNoiseParameters;
         }

         @Override
         public double getEstimatorDT()
         {
            return controlDT;
         }

         @Override
         public void configureSensorProcessing(SensorProcessing sensorProcessing)
         {
            DoubleYoVariable alphaFilter = sensorProcessing.createAlphaFilter("defaultAlphaFilter", 12.0);

            sensorProcessing.addSensorAlphaFilter(alphaFilter, false, SensorType.JOINT_POSITION);
            sensorProcessing.addSensorAlphaFilter(alphaFilter, false, SensorType.JOINT_VELOCITY);
            sensorProcessing.addSensorAlphaFilter(alphaFilter, false, SensorType.IMU_ORIENTATION);
            sensorProcessing.addSensorAlphaFilter(alphaFilter, false, SensorType.IMU_ANGULAR_VELOCITY);
            sensorProcessing.addSensorAlphaFilter(alphaFilter, false, SensorType.IMU_LINEAR_ACCELERATION);
         }
      };
      SensorReaderFactory simulatedSensorHolderAndReaderFromRobotFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(robot,
            sensorProcessingConfiguration);

      simulatedSensorHolderAndReaderFromRobotFactory.build(inverseDynamicsStructure.getRootJoint(), null, null, null,null, null, registry);

      SensorReader simulatedSensorHolderAndReader = simulatedSensorHolderAndReaderFromRobotFactory.getSensorReader();

      Joint estimationJoint = robot.getRootJoint();
      robot.update();

      ReferenceFrame estimationFrame = inverseDynamicsStructure.getEstimationFrame();

      double comAccelerationProcessNoiseStandardDeviation = simulatedSensorNoiseParameters.getComAccelerationProcessNoiseStandardDeviation();
      double angularAccelerationProcessNoiseStandardDeviation = simulatedSensorNoiseParameters.getAngularAccelerationProcessNoiseStandardDeviation();
      DesiredCoMAccelerationsFromRobotStealerController desiredCoMAccelerationsFromRobotStealerController = new DesiredCoMAccelerationsFromRobotStealerController(
            estimationFrame, comAccelerationProcessNoiseStandardDeviation, angularAccelerationProcessNoiseStandardDeviation, generator, estimationJoint,
            controlDT);

      double gravitationalAcceleration = robot.getGravityZ();

      // The following few lines are what you need to do to get the state estimator working with a robot.
      // You also need to either add the controlFlowGraph to another one, or make sure to run it's startComputation method at the right time:
      //      SensorNoiseParameters sensorNoiseParametersForEstimator = SensorNoiseParametersForEvaluator.createVeryLittleSensorNoiseParameters();
      //      SensorNoiseParameters sensorNoiseParametersForEstimator = SensorNoiseParametersForEvaluator.createSensorNoiseParameters();
      //      SensorNoiseParameters sensorNoiseParametersForEstimator = SensorNoiseParametersForEvaluator.createLotsOfSensorNoiseParameters();
      final SensorNoiseParameters sensorNoiseParametersForEstimator = SensorNoiseParametersForEvaluator.createTunedNoiseParametersForEvaluator();

      RigidBodyToIndexMap rigidBodyToIndexMap = new RigidBodyToIndexMap(inverseDynamicsStructure.getElevator());

      AfterJointReferenceFrameNameMap referenceFrameMap = new AfterJointReferenceFrameNameMap(inverseDynamicsStructure.getElevator());
      final StateEstimationDataFromController stateEstimatorDataFromControllerSource = new StateEstimationDataFromController(estimationFrame);
      final StateEstimationDataFromController stateEstimationDataFromControllerSink = new StateEstimationDataFromController(estimationFrame);

      double pointVelocityXYMeasurementStandardDeviation = 2.0;
      double pointPositionZMeasurementStandardDeviation = 0.1;
      double pointVelocityZMeasurementStandardDeviation = 2.0;
      double pointPositionXYMeasurementStandardDeviation = 0.1;

      final PointMeasurementNoiseParameters pointMeasurementNoiseParameters = new PointMeasurementNoiseParameters(pointVelocityXYMeasurementStandardDeviation,
            pointVelocityZMeasurementStandardDeviation, pointPositionXYMeasurementStandardDeviation, pointPositionZMeasurementStandardDeviation);

      StateEstimatorParameters stateEstimatorParameters = new StateEstimatorParameters()
      {
         public boolean isRunningOnRealRobot()
         {
            return false;
         }

         public SensorNoiseParameters getSensorNoiseParameters()
         {
            return sensorNoiseParametersForEstimator;
         }

         public double getEstimatorDT()
         {
            return controlDT;
         }

         public double getKinematicsPelvisPositionFilterFreqInHertz()
         {
            return 0;
         }

         public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
         {
            return 0;
         }

         public double getCoPFilterFreqInHertz()
         {
            return 0;
         }

         public boolean useAccelerometerForEstimation()
         {
            return false;
         }

         public boolean enableIMUBiasCompensation()
         {
            return false;
         }

         @Override
         public boolean cancelGravityFromAccelerationMeasurement()
         {
            return true;
         }

         public double getIMUBiasFilterFreqInHertz()
         {
            return 0;
         }

         public double getPelvisPositionFusingFrequency()
         {
            return 0;
         }

         public double getPelvisLinearVelocityFusingFrequency()
         {
            return 0;
         }

         public double getDelayTimeForTrustingFoot()
         {
            return 0;
         }

         public double getForceInPercentOfWeightThresholdToTrustFoot()
         {
            return 0;
         }

         public boolean enableIMUYawDriftCompensation()
         {
            return false;
         }

         public double getIMUYawDriftFilterFreqInHertz()
         {
            return 0;
         }


         @Override
         public double getIMUBiasVelocityThreshold()
         {
            return 0.015;
         }

         public boolean trustCoPAsNonSlippingContactPoint()
         {
            return true;
         }

         @Override
         public double getPelvisLinearVelocityAlphaNewTwist()
         {
            return 1.0;
         }

         public double getContactThresholdForce()
         {
            return 120.0;
         }

         @Override
         public double getFootSwitchCoPThresholdFraction()
         {
            return 0.02;
         }

         @Override
         public void configureSensorProcessing(SensorProcessing newSensorProcessing)
         {
            // TODO Auto-generated method stub

         }

         @Override
         public double getContactThresholdHeight()
         {
            // TODO Auto-generated method stub
            return 0;
         }

         @Override
         public FootSwitchType getFootSwitchType()
         {
            return null;
         }

         @Override
         public boolean requestFootForceSensorCalibrationAtStart()
         {
            // TODO Auto-generated method stub
            return false;
         }

         @Override
         public SideDependentList<String> getFootForceSensorNames()
         {
            // TODO Auto-generated method stub
            return null;
         }

         @Override
         public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
         {
            // TODO Auto-generated method stub
            return false;
         }

         @Override
         public double getCenterOfMassVelocityFusingFrequency()
         {
            return 0.4261;
         }

         @Override
         public boolean useGroundReactionForcesToComputeCenterOfMassVelocity()
         {
            return false;
         }
      };

      JointAndIMUSensorMap sensorMap = null; // simulatedSensorHolderAndReaderFromRobotFactory.getSensorReader().getJointAndIMUSensorMap();
      System.err.println("This state estimator is broken! Reimplement me!");
      System.exit(-1);
      SensorAndEstimatorAssembler sensorAndEstimatorAssembler = new SensorAndEstimatorAssembler(stateEstimatorDataFromControllerSource, sensorMap,
            stateEstimatorParameters, gravitationalAcceleration, inverseDynamicsStructure, referenceFrameMap, rigidBodyToIndexMap, registry);

      ControlFlowGraph controlFlowGraph = sensorAndEstimatorAssembler.getControlFlowGraph();
      StateEstimatorWithPorts orientationEstimator = sensorAndEstimatorAssembler.getEstimator();

      desiredCoMAccelerationsFromRobotStealerController.attachStateEstimationDataFromControllerSink(stateEstimationDataFromControllerSink);

      ControlFlowGraphExecutorController controlFlowGraphExecutorController = new ControlFlowGraphExecutorController(controlFlowGraph);

      StateEstimatorErrorCalculatorController composableStateEstimatorEvaluatorController = new StateEstimatorErrorCalculatorController(orientationEstimator,
            robot, estimationJoint, assumePerfectIMU, useSimpleComEstimator);

      robot.setController(desiredCoMAccelerationsFromRobotStealerController, simTicksPerControlDT);
      RunnableRunnerController runnableRunnerController = new RunnableRunnerController();
      runnableRunnerController.addRunnable((Runnable) simulatedSensorHolderAndReader);
      runnableRunnerController.addRunnable(new Runnable()
      {

         public void run()
         {
            stateEstimatorDataFromControllerSource.set(stateEstimationDataFromControllerSink);
         }
      });
      robot.setController(runnableRunnerController, simTicksPerControlDT);

      robot.setController(controlFlowGraphExecutorController, simTicksPerControlDT);
      robot.setController(composableStateEstimatorEvaluatorController, simTicksPerControlDT);

      if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
      {
         boolean updateRootJoints = true;
         boolean updateDesireds = false;
         generator.updateInverseDynamicsRobotModelFromRobot(updateRootJoints, updateDesireds);

         RotationMatrix rotationMatrix = new RotationMatrix();
         robot.getRootJoint().getRotationToWorld(rotationMatrix);
         Vector3D angularVelocityInBody = robot.getRootJoint().getAngularVelocityInBody();

         FrameOrientation estimatedOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
         orientationEstimator.getEstimatedOrientation(estimatedOrientation);
         estimatedOrientation.set(rotationMatrix);
         orientationEstimator.setEstimatedOrientation(estimatedOrientation);

         FrameVector estimatedAngularVelocity = new FrameVector();
         orientationEstimator.getEstimatedAngularVelocity(estimatedAngularVelocity);
         estimatedAngularVelocity.set(angularVelocityInBody);
         orientationEstimator.setEstimatedAngularVelocity(estimatedAngularVelocity);

         // TODO: This wasn't doing anything.
         // DenseMatrix64F x = orientationEstimator.getState();
         // MatrixTools.insertTuple3dIntoEJMLVector(angularVelocityInBody, x, 3);
         // orientationEstimator.setState(x, orientationEstimator.getCovariance());
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(SHOW_GUI);
      parameters.setDataBufferSize(32000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
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
