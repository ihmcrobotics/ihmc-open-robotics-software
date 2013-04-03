package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.dynamics.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.signalCorruption.BiasVectorCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.signalCorruption.GaussianVectorCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedAngularVelocitySensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedLinearAccelerationSensor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedOrientationSensor;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassAccelerationCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.FloatingJoint;
import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.Link;
import com.yobotics.simulationconstructionset.PinJoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class ComposableStateEstimatorEvaluator
{
   private static final boolean INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL = true; //false;
   private static final boolean USE_ANGULAR_ACCELERATION_INPUT = true;

   private static final boolean CREATE_ORIENTATION_SENSOR = true;
   private static final boolean CREATE_ANGULAR_VELOCITY_SENSOR = true;
   private static final boolean CREATE_LINEAR_ACCELERATION_SENSOR = true;

   private static final boolean ESTIMATE_COM = true;
   private static final boolean ADD_ARM_LINKS = true;

   private final double orientationMeasurementStandardDeviation = Math.sqrt(1e-2);
   private final double angularVelocityMeasurementStandardDeviation = Math.sqrt(1e-5);
   private final double linearAccelerationMeasurementStandardDeviation = Math.sqrt(1e-2);

   private final double angularAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double comAccelerationProcessNoiseStandardDeviation = Math.sqrt(1e-1);
   private final double angularVelocityBiasProcessNoiseStandardDeviation = Math.sqrt(1e-4);
   private final double linearAccelerationBiasProcessNoiseStandardDeviation = Math.sqrt(1e-4);

   private final Vector3d gravitationalAccelerationForSimulation = new Vector3d(0.0, 0.0, 0.0);
   private final Vector3d gravitationalAccelerationForSensors = new Vector3d(0.0, 0.0, -9.81);
   
   private final double simDT = 1e-3;
   private final int simTicksPerControlDT = 5;
   private final double controlDT = simDT * simTicksPerControlDT;
   private final int simTicksPerRecord = simTicksPerControlDT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   public ComposableStateEstimatorEvaluator()
   {
      StateEstimatorEstimatorEvaluatorRobot robot = new StateEstimatorEstimatorEvaluatorRobot();
      QuaternionOrientationEstimatorEvaluatorController controller = new QuaternionOrientationEstimatorEvaluatorController(robot, controlDT);
      robot.setController(controller, simTicksPerControlDT);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, true, 32000);
      scs.addYoVariableRegistry(registry);

      scs.setDT(simDT, simTicksPerRecord);
      scs.setSimulateDuration(45.0);
      scs.startOnAThread();
      scs.simulate();
   }

   private class StateEstimatorEstimatorEvaluatorRobot extends Robot
   {
      private static final long serialVersionUID = 2647791981594204134L;
      private final Link bodyLink;
      private final FloatingJoint rootJoint;
      private final ArrayList<IMUMount> imuMounts = new ArrayList<IMUMount>();

      public StateEstimatorEstimatorEvaluatorRobot()
      {
         super(StateEstimatorEstimatorEvaluatorRobot.class.getSimpleName());

         rootJoint = new FloatingJoint("root", new Vector3d(), this);

         bodyLink = new Link("body");
         bodyLink.setMassAndRadiiOfGyration(10.0, 0.1, 0.2, 0.3);

         ExternalForcePoint externalForcePoint = new ExternalForcePoint("ef_rootJoint", this);
         rootJoint.addExternalForcePoint(externalForcePoint);

         Graphics3DObject bodyLinkGraphics = new Graphics3DObject();
         bodyLinkGraphics.translate(0.0, 0.0, -0.15);
         bodyLinkGraphics.addCube(0.1, 0.2, 0.3, YoAppearance.Red());
         bodyLink.setLinkGraphics(bodyLinkGraphics);
         rootJoint.setLink(bodyLink);

         Transform3D imu0Offset = new Transform3D();
         IMUMount imuMount0 = new IMUMount("imuMount0", imu0Offset, this);
         rootJoint.addIMUMount(imuMount0);

         this.addRootJoint(rootJoint);

         if (ADD_ARM_LINKS)
         {
            PinJoint pinJoint1 = new PinJoint("pinJoint1", new Vector3d(), this, Axis.X);
            Link armLink1 = new Link("armLink1");
            armLink1.setMassAndRadiiOfGyration(0.3, 0.1, 0.1, 0.1);
            armLink1.setComOffset(new Vector3d(0.0, 0.0, 0.5));

            Graphics3DObject armLink1Graphics = new Graphics3DObject();
            //            armLink1Graphics.rotate(-Math.PI/2.0, Graphics3DObject.X);
            armLink1Graphics.addCylinder(1.0, 0.02, YoAppearance.Green());
            armLink1.setLinkGraphics(armLink1Graphics);
            pinJoint1.setLink(armLink1);

            Transform3D imu1Offset = new Transform3D();
            IMUMount imuMount1 = new IMUMount("imuMount1", imu1Offset, this);
            pinJoint1.addIMUMount(imuMount1);

            rootJoint.addJoint(pinJoint1);

            PinJoint pinJoint2 = new PinJoint("pinJoint2", new Vector3d(0.0, 0.0, 1.0), this, Axis.Z);
            Link armLink2 = new Link("armLink2");
            armLink2.setMassAndRadiiOfGyration(0.2, 0.1, 0.1, 0.1);
            armLink2.setComOffset(new Vector3d(0.5, 0.0, 0.0));

            Graphics3DObject armLink2Graphics = new Graphics3DObject();
            armLink2Graphics.rotate(Math.PI / 2.0, Graphics3DObject.Y);
            armLink2Graphics.addCylinder(1.0, 0.02, YoAppearance.Blue());
            armLink2.setLinkGraphics(armLink2Graphics);
            pinJoint2.setLink(armLink2);

            Transform3D imu2Offset = new Transform3D();
            imu2Offset.rotY(Math.PI / 8.0);
            imu2Offset.setTranslation(new Vector3d(0.0, 0.0, 0.1));
            IMUMount imuMount2 = new IMUMount("imuMount2", imu2Offset, this);
            pinJoint2.addIMUMount(imuMount2);

            pinJoint1.addJoint(pinJoint2);

            pinJoint1.setQ(1.2);
            pinJoint2.setQ(0.8);

            pinJoint1.setQd(-0.5);
            pinJoint2.setQd(0.77);

            imuMounts.add(imuMount0);
            imuMounts.add(imuMount1);
            imuMounts.add(imuMount2);
         }

         else
         {
            imuMounts.add(imuMount0);
         }

         this.setGravity(gravitationalAccelerationForSimulation);

         if (ADD_ARM_LINKS)
         {
            rootJoint.setPosition(new Point3d(0.0, 0.0, 0.4));
            Matrix3d rotationMatrix = new Matrix3d();
            rotationMatrix.rotX(0.6);
            rootJoint.setRotation(rotationMatrix);

            rootJoint.setAngularVelocityInBody(new Vector3d(0.2, 2.2, 0.3));
         }

         else
         {
            rootJoint.setPosition(new Point3d(0.0, 0.0, 0.4));
            rootJoint.setAngularVelocityInBody(new Vector3d(0.2, 2.5, 0.0));
         }

         update();
      }

      //      public Link getBodyLink()
      //      {
      //         return bodyLink;
      //      }

      public FloatingJoint getRootJoint()
      {
         return rootJoint;
      }

      public ArrayList<IMUMount> getIMUMounts()
      {
         return imuMounts;
      }

      public Vector3d getActualAngularAccelerationInBodyFrame()
      {
         return rootJoint.getAngularAccelerationInBody();
      }

      public Quat4d getActualOrientation()
      {
         return rootJoint.getQuaternion();
      }
   }

   private class StateEstimatorEvaluatorFullRobotModel
   {
      private final InverseDynamicsJointsFromSCSRobotGenerator generator;

      private final RigidBody elevator;
      private final SixDoFJoint rootInverseDynamicsJoint;
      private final RigidBody rootBody;

      private final ArrayList<IMUMount> imuMounts;

      public StateEstimatorEvaluatorFullRobotModel(StateEstimatorEstimatorEvaluatorRobot robot)
      {
         generator = new InverseDynamicsJointsFromSCSRobotGenerator(robot);
         elevator = generator.getElevator();

         rootInverseDynamicsJoint = generator.getRootSixDoFJoint();
         rootBody = generator.getRootBody();

         imuMounts = robot.getIMUMounts();
      }

      public SixDoFJoint getRootInverseDynamicsJoint()
      {
         return rootInverseDynamicsJoint;
      }

      public RigidBody getRootBody()
      {
         return rootBody;
      }

      public ArrayList<IMUMount> getIMUMounts()
      {
         return imuMounts;
      }

      public RigidBody getIMUBody(IMUMount imuMount)
      {
         return generator.getRigidBody(imuMount.getParentJoint());
      }

      public void updateBasedOnRobot(StateEstimatorEstimatorEvaluatorRobot robot, boolean updateRootJoints)
      {
         generator.updateInverseDynamicsRobotModelFromRobot(updateRootJoints, false);
      }

      public void updateBasedOnEstimator(OrientationEstimator estimator)
      {
         FrameOrientation estimatedOrientation = estimator.getEstimatedOrientation();
         FrameVector estimatedAngularVelocity = estimator.getEstimatedAngularVelocity();

         updateBasedOnEstimator(estimatedOrientation, estimatedAngularVelocity);
      }

      public void updateBasedOnEstimator(FrameOrientation estimatedOrientation, FrameVector estimatedAngularVelocity)
      {
         rootInverseDynamicsJoint.setRotation(estimatedOrientation.getQuaternion());
         generator.updateInverseDynamicsRobotModelFromRobot(false, false);

         elevator.updateFramesRecursively();

         ReferenceFrame elevatorFrame = rootInverseDynamicsJoint.getFrameBeforeJoint();
         ReferenceFrame bodyFrame = rootInverseDynamicsJoint.getFrameAfterJoint();

         estimatedAngularVelocity.changeFrame(bodyFrame);

         Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame);
         bodyTwist.setAngularPart(estimatedAngularVelocity.getVector());
         rootInverseDynamicsJoint.setJointTwist(bodyTwist);
      }
   }

   private class AngularAccelerationFromRobotStealer extends AbstractControlFlowElement
   {
      private final StateEstimatorEstimatorEvaluatorRobot robot;
      private final ControlFlowOutputPort<FrameVector> outputPort = createOutputPort();
      private final FrameVector desiredAngularAcceleration;
      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public AngularAccelerationFromRobotStealer(StateEstimatorEstimatorEvaluatorRobot robot, ReferenceFrame referenceFrame)
      {
         this.robot = robot;
         this.desiredAngularAcceleration = new FrameVector(referenceFrame);
         double discreteStdDev = convertStandardDeviationToDiscreteTime(angularAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public void startComputation()
      {
         desiredAngularAcceleration.set(robot.getActualAngularAccelerationInBodyFrame());
         signalCorruptor.corrupt(desiredAngularAcceleration.getVector());
         outputPort.setData(desiredAngularAcceleration);
      }

      public ControlFlowOutputPort<FrameVector> getOutputPort()
      {
         return outputPort;
      }

      public void waitUntilComputationIsDone()
      {
      }

   }

   private class CenterOfMassAccelerationFromFullRobotModelStealer extends AbstractControlFlowElement
   {
      private final CenterOfMassAccelerationCalculator centerOfMassAccelerationCalculator;
      private final FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      private final ControlFlowOutputPort<FrameVector> outputPort = createOutputPort();
      private final GaussianVectorCorruptor signalCorruptor = new GaussianVectorCorruptor(123412L, getClass().getSimpleName() + "Corruptor", registry);

      public CenterOfMassAccelerationFromFullRobotModelStealer(RigidBody rootBody, SpatialAccelerationCalculator spatialAccelerationCalculator)
      {
         this.centerOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(rootBody, spatialAccelerationCalculator);

         double discreteStdDev = convertStandardDeviationToDiscreteTime(comAccelerationProcessNoiseStandardDeviation);
         signalCorruptor.setStandardDeviation(discreteStdDev);
      }

      public ControlFlowOutputPort<FrameVector> getOutputPort()
      {
         return outputPort;
      }

      public void startComputation()
      {
         centerOfMassAccelerationCalculator.packCoMAcceleration(comAcceleration);
         comAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
         signalCorruptor.corrupt(comAcceleration.getVector());
         outputPort.setData(comAcceleration);
      }

      public void waitUntilComputationIsDone()
      {
      }
   }

   private class QuaternionOrientationEstimatorEvaluatorController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("QuaternionOrientationEstimatorEvaluatorController");

      private final DoubleYoVariable orientationErrorAngle = new DoubleYoVariable("orientationErrorAngle", registry);

      private final StateEstimatorEstimatorEvaluatorRobot robot;

      private final StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel;
      private final TwistCalculator perfectTwistCalculator;
      private final SpatialAccelerationCalculator perfectSpatialAccelerationCalculator;

      private final StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel;
      private final TwistCalculator estimatedTwistCalculator;
      private final SpatialAccelerationCalculator estimatedSpatialAccelerationCalculator;

      private final CenterOfMassCalculator perfectCenterOfMassCalculator;
      private final CenterOfMassJacobian perfectCenterOfMassJacobian;
      private final CenterOfMassAccelerationCalculator perfectCenterOfMassAccelerationCalculator;
      
      private final YoFramePoint perfectCoM = new YoFramePoint("perfectCoM", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector perfectCoMd = new YoFrameVector("perfectCoMd", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector perfectCoMdd = new YoFrameVector("perfectCoMdd", ReferenceFrame.getWorldFrame(), registry);
      
      private final ControlFlowGraph controlFlowGraph;
      private final OrientationEstimator orientationEstimator;

      public QuaternionOrientationEstimatorEvaluatorController(StateEstimatorEstimatorEvaluatorRobot robot, double controlDT)
      {
         this.robot = robot;

         perfectFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(robot);
         perfectTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), perfectFullRobotModel.elevator);

         perfectSpatialAccelerationCalculator = new SpatialAccelerationCalculator(perfectFullRobotModel.elevator, perfectTwistCalculator, 0.0, false);
         estimatedFullRobotModel = new StateEstimatorEvaluatorFullRobotModel(robot);
         estimatedTwistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), estimatedFullRobotModel.elevator);
         estimatedSpatialAccelerationCalculator = new SpatialAccelerationCalculator(estimatedFullRobotModel.elevator, estimatedTwistCalculator, 0.0, false);

         perfectCenterOfMassCalculator = new CenterOfMassCalculator(perfectFullRobotModel.elevator, ReferenceFrame.getWorldFrame());
         perfectCenterOfMassJacobian = new CenterOfMassJacobian(perfectFullRobotModel.elevator);
         perfectCenterOfMassAccelerationCalculator = new CenterOfMassAccelerationCalculator(perfectFullRobotModel.elevator, perfectSpatialAccelerationCalculator);

         ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = createOrientationSensors(perfectFullRobotModel, estimatedFullRobotModel);
         ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = createAngularVelocitySensors(perfectFullRobotModel,
               estimatedFullRobotModel);
         ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = createLinearAccelerationSensors(perfectFullRobotModel,
               estimatedFullRobotModel);

         controlFlowGraph = new ControlFlowGraph();
         RigidBody estimationLink = estimatedFullRobotModel.getRootBody();
         ReferenceFrame estimationFrame = estimationLink.getParentJoint().getFrameAfterJoint();

         DenseMatrix64F angularAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(angularAccelerationProcessNoiseStandardDeviation, 3);

         ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort = null;
         if (ESTIMATE_COM || USE_ANGULAR_ACCELERATION_INPUT)
         {
            AngularAccelerationFromRobotStealer angularAccelerationFromRobotStealer = new AngularAccelerationFromRobotStealer(robot, estimationFrame);
            desiredAngularAccelerationOutputPort = angularAccelerationFromRobotStealer.getOutputPort();
         }

         if (ESTIMATE_COM)
         {
            CenterOfMassAccelerationFromFullRobotModelStealer centerOfMassAccelerationFromFullRobotModelStealer = new CenterOfMassAccelerationFromFullRobotModelStealer(
                  perfectFullRobotModel.elevator, perfectSpatialAccelerationCalculator);
            ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort = centerOfMassAccelerationFromFullRobotModelStealer.getOutputPort();

            DenseMatrix64F comAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(comAccelerationProcessNoiseStandardDeviation, 3);

            ComposableOrientationAndCoMEstimatorCreator orientationEstimatorCreator = new ComposableOrientationAndCoMEstimatorCreator(
                  angularAccelerationNoiseCovariance, comAccelerationNoiseCovariance, estimationLink, estimatedTwistCalculator,
                  estimatedSpatialAccelerationCalculator);
            orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
            orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);
            orientationEstimatorCreator.addLinearAccelerationSensorConfigurations(linearAccelerationSensorConfigurations);

            updateInternalState();
            orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT,
                  estimatedFullRobotModel.getRootInverseDynamicsJoint(), estimationLink, estimationFrame, desiredAngularAccelerationOutputPort,
                  desiredCenterOfMassAccelerationOutputPort, registry);
         }
         else
         {
            ComposableOrientationEstimatorCreator orientationEstimatorCreator = new ComposableOrientationEstimatorCreator(angularAccelerationNoiseCovariance,
                  estimationLink, estimatedTwistCalculator);
            orientationEstimatorCreator.addOrientationSensorConfigurations(orientationSensorConfigurations);
            orientationEstimatorCreator.addAngularVelocitySensorConfigurations(angularVelocitySensorConfigurations);

            orientationEstimator = orientationEstimatorCreator.createOrientationEstimator(controlFlowGraph, controlDT, estimationFrame,
                  desiredAngularAccelerationOutputPort, registry);
         }

         controlFlowGraph.initializeAfterConnections();

         if (INITIALIZE_ANGULAR_VELOCITY_ESTIMATE_TO_ACTUAL)
         {
            robot.update();
            estimatedFullRobotModel.updateBasedOnRobot(robot, true);

            Matrix3d rotationMatrix = new Matrix3d();
            robot.getRootJoint().getRotationToWorld(rotationMatrix);
            Vector3d angularVelocityInBody = robot.getRootJoint().getAngularVelocityInBody();

            FrameOrientation estimatedOrientation = orientationEstimator.getEstimatedOrientation();
            estimatedOrientation.set(rotationMatrix);
            orientationEstimator.setEstimatedOrientation(estimatedOrientation);

            FrameVector estimatedAngularVelocity = orientationEstimator.getEstimatedAngularVelocity();
            estimatedAngularVelocity.set(angularVelocityInBody);
            orientationEstimator.setEstimatedAngularVelocity(estimatedAngularVelocity);

            //TODO: This wasn't doing anything. 
            //            DenseMatrix64F x = orientationEstimator.getState();
            //            MatrixTools.insertTuple3dIntoEJMLVector(angularVelocityInBody, x, 3);
            //            orientationEstimator.setState(x, orientationEstimator.getCovariance());

            System.out.println("Estimated orientation = " + orientationEstimator.getEstimatedOrientation());
            System.out.println("Estimated angular velocity = " + estimatedAngularVelocity);
         }

      }

      private ArrayList<AngularVelocitySensorConfiguration> createAngularVelocitySensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
            StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
      {
         ArrayList<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();

         if (CREATE_ANGULAR_VELOCITY_SENSOR)
         {
            for (IMUMount imuMount : perfectFullRobotModel.getIMUMounts())
            {
               RigidBody perfectMeasurmentBody = perfectFullRobotModel.getIMUBody(imuMount);
               RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getIMUBody(imuMount);

               ReferenceFrame frameUsedForPerfectMeasurement = perfectMeasurmentBody.getParentJoint().getFrameAfterJoint();
               String sensorName = imuMount.getName() + "AngularVelocity";

               SimulatedAngularVelocitySensor angularVelocitySensor = new SimulatedAngularVelocitySensor(sensorName, perfectTwistCalculator,
                     perfectMeasurmentBody, frameUsedForPerfectMeasurement, registry);
               GaussianVectorCorruptor angularVelocityCorruptor = new GaussianVectorCorruptor(1235L, sensorName, registry);
               angularVelocityCorruptor.setStandardDeviation(angularVelocityMeasurementStandardDeviation);
               angularVelocitySensor.addSignalCorruptor(angularVelocityCorruptor);

               BiasVectorCorruptor biasVectorCorruptor = new BiasVectorCorruptor(1236L, sensorName, controlDT, registry);
               biasVectorCorruptor.setStandardDeviation(angularVelocityBiasProcessNoiseStandardDeviation);
               biasVectorCorruptor.setBias(new Vector3d(0.0, 0.0, 0.0));
               angularVelocitySensor.addSignalCorruptor(biasVectorCorruptor);

               DenseMatrix64F angularVelocityNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityMeasurementStandardDeviation, 3);
               DenseMatrix64F angularVelocityBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(angularVelocityBiasProcessNoiseStandardDeviation, 3);

               ReferenceFrame measurementFrame = estimatedMeasurementBody.getParentJoint().getFrameAfterJoint();

               AngularVelocitySensorConfiguration angularVelocitySensorConfiguration = new AngularVelocitySensorConfiguration(
                     angularVelocitySensor.getAngularVelocityOutputPort(), sensorName, estimatedMeasurementBody, measurementFrame,
                     angularVelocityNoiseCovariance, angularVelocityBiasProcessNoiseCovariance);
               angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
            }
         }

         return angularVelocitySensorConfigurations;
      }

      private ArrayList<LinearAccelerationSensorConfiguration> createLinearAccelerationSensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
            StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
      {
         ArrayList<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

         if (CREATE_LINEAR_ACCELERATION_SENSOR)
         {
            for (IMUMount imuMount : perfectFullRobotModel.getIMUMounts())
            {
               RigidBody perfectMeasurmentBody = perfectFullRobotModel.getIMUBody(imuMount);
               RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getIMUBody(imuMount);

               ReferenceFrame frameUsedForPerfectMeasurement = perfectMeasurmentBody.getParentJoint().getFrameAfterJoint();
               String sensorName = imuMount.getName() + "LinearAcceleration";

               SimulatedLinearAccelerationSensor linearAccelerationSensor = new SimulatedLinearAccelerationSensor(sensorName, perfectMeasurmentBody,
                     frameUsedForPerfectMeasurement, perfectSpatialAccelerationCalculator, gravitationalAccelerationForSensors, registry);

               GaussianVectorCorruptor linearAccelerationCorruptor = new GaussianVectorCorruptor(1237L, sensorName, registry);
               linearAccelerationCorruptor.setStandardDeviation(linearAccelerationMeasurementStandardDeviation);
               linearAccelerationSensor.addSignalCorruptor(linearAccelerationCorruptor);

               BiasVectorCorruptor biasVectorCorruptor = new BiasVectorCorruptor(1286L, sensorName, controlDT, registry);
               biasVectorCorruptor.setStandardDeviation(linearAccelerationBiasProcessNoiseStandardDeviation);
               biasVectorCorruptor.setBias(new Vector3d(0.0, 0.0, 0.0));
               linearAccelerationSensor.addSignalCorruptor(biasVectorCorruptor);

               DenseMatrix64F linearAccelerationNoiseCovariance = createDiagonalCovarianceMatrix(linearAccelerationMeasurementStandardDeviation, 3);
               DenseMatrix64F linearAccelerationBiasProcessNoiseCovariance = createDiagonalCovarianceMatrix(
                     linearAccelerationBiasProcessNoiseStandardDeviation, 3);

               ReferenceFrame measurementFrame = estimatedMeasurementBody.getParentJoint().getFrameAfterJoint();

               LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration = new LinearAccelerationSensorConfiguration(
                     linearAccelerationSensor.getLinearAccelerationOutputPort(), sensorName, estimatedMeasurementBody, measurementFrame, -gravitationalAccelerationForSensors.getZ(),
                     linearAccelerationNoiseCovariance, linearAccelerationBiasProcessNoiseCovariance);

               linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
            }
         }

         return linearAccelerationSensorConfigurations;
      }

      private ArrayList<OrientationSensorConfiguration> createOrientationSensors(StateEstimatorEvaluatorFullRobotModel perfectFullRobotModel,
            StateEstimatorEvaluatorFullRobotModel estimatedFullRobotModel)
      {
         ArrayList<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();

         if (CREATE_ORIENTATION_SENSOR)
         {
            for (IMUMount imuMount : perfectFullRobotModel.getIMUMounts())
            {
               RigidBody perfectMeasurementBody = perfectFullRobotModel.getIMUBody(imuMount);
               RigidBody estimatedMeasurementBody = estimatedFullRobotModel.getIMUBody(imuMount);

               ReferenceFrame frameUsedForPerfectMeasurement = perfectMeasurementBody.getParentJoint().getFrameAfterJoint();
               String sensorName = imuMount.getName() + "Orientation";

               SimulatedOrientationSensor sensor = new SimulatedOrientationSensor(sensorName, frameUsedForPerfectMeasurement, registry);
               GaussianOrientationCorruptor orientationCorruptor = new GaussianOrientationCorruptor(sensorName, 12334255L, registry);
               orientationCorruptor.setStandardDeviation(orientationMeasurementStandardDeviation);
               sensor.addSignalCorruptor(orientationCorruptor);

               DenseMatrix64F orientationNoiseCovariance = createDiagonalCovarianceMatrix(orientationMeasurementStandardDeviation, 3);

               ReferenceFrame measurementFrame = estimatedMeasurementBody.getParentJoint().getFrameAfterJoint();

               OrientationSensorConfiguration orientationSensorConfiguration = new OrientationSensorConfiguration(sensor.getOrientationOutputPort(),
                     sensorName, measurementFrame, orientationNoiseCovariance);
               orientationSensorConfigurations.add(orientationSensorConfiguration);
            }
         }

         return orientationSensorConfigurations;
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
         perfectFullRobotModel.updateBasedOnRobot(robot, true);
         perfectTwistCalculator.compute();
         perfectSpatialAccelerationCalculator.compute();

         updateInternalState();
         updateGroundTruth();

         controlFlowGraph.startComputation();
         controlFlowGraph.waitUntilComputationIsDone();

         if (!ESTIMATE_COM) // this is being done inside the state estimator for the CoM estimator
            estimatedFullRobotModel.updateBasedOnEstimator(orientationEstimator);

         estimatedTwistCalculator.compute();

         computeOrientationErrorAngle();
      }

      private void updateGroundTruth()
      {
         FramePoint com = new FramePoint();
         perfectCenterOfMassCalculator.compute();
         perfectCenterOfMassCalculator.packCenterOfMass(com);
         perfectCoM.set(com);
         
         FrameVector comd = new FrameVector();
         perfectCenterOfMassJacobian.compute();
         perfectCenterOfMassJacobian.packCenterOfMassVelocity(comd);
         comd.changeFrame(ReferenceFrame.getWorldFrame());
         perfectCoMd.set(comd);
         
         FrameVector comdd = new FrameVector();
         perfectCenterOfMassAccelerationCalculator.packCoMAcceleration(comdd);
         comdd.changeFrame(ReferenceFrame.getWorldFrame());
         perfectCoMdd.set(comdd);
      }

      private void updateInternalState()
      {
         /*
          * supersense joint positions and velocities and update twist
          * calculator and spatial accel calculator based on this data TODO:
          * need a class that does this based on sensors
          */
         estimatedFullRobotModel.updateBasedOnRobot(robot, false);
         estimatedTwistCalculator.compute();
         estimatedSpatialAccelerationCalculator.compute();
      }

      private void computeOrientationErrorAngle()
      {
         FrameOrientation estimatedOrientation = orientationEstimator.getEstimatedOrientation();
         Quat4d estimatedOrientationQuat4d = new Quat4d();
         estimatedOrientation.getQuaternion(estimatedOrientationQuat4d);

         Quat4d orientationErrorQuat4d = new Quat4d(robot.getActualOrientation());
         orientationErrorQuat4d.mulInverse(estimatedOrientationQuat4d);

         AxisAngle4d orientationErrorAxisAngle = new AxisAngle4d();
         orientationErrorAxisAngle.set(orientationErrorQuat4d);

         double errorAngle = AngleTools.trimAngleMinusPiToPi(orientationErrorAxisAngle.getAngle());

         orientationErrorAngle.set(Math.abs(errorAngle));
      }
   }

   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
      CommonOps.setIdentity(orientationCovarianceMatrix);
      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);

      return orientationCovarianceMatrix;
   }

   public static void main(String[] args)
   {
      new ComposableStateEstimatorEvaluator();
   }

   private double convertStandardDeviationToDiscreteTime(double continuousStdDev)
   {
      double continuousVariance = MathTools.square(continuousStdDev);
      double discreteVariance = continuousVariance * controlDT;
      double discreteStdDev = Math.sqrt(discreteVariance);
      return discreteStdDev;
   }
}
