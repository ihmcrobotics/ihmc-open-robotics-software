package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFramePointControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.AggregatePointPositionMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.AggregatePointVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.LinearAccelerationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.OrientationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AbstractProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AngularAccelerationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassAccelerationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassPositionDiscreteProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassPositionProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassVelocityDiscreteProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassVelocityProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.ProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableOrientationAndCoMEstimatorCreator
{
   //These are tuned for PointPositionGrabber:
   private static final double pointVelocityXYMeasurementStandardDeviation = 6.0;
   private static final double pointVelocityZMeasurementStandardDeviation = 6.0;

   private static final double pointPositionXYMeasurementStandardDeviation = 0.3;
   private static final double pointPositionZMeasurementStandardDeviation = 0.3;

   // These are tuned for SingleReferenceFramePointPositionGrabber:
   //   private static final double pointVelocityXYMeasurementStandardDeviation = 1.5; 
   //   private static final double pointVelocityZMeasurementStandardDeviation = 1.5;

   //   private static final double pointPositionXYMeasurementStandardDeviation = 0.1; 
   //   private static final double pointPositionZMeasurementStandardDeviation = 0.1; 

   private static final boolean USE_DISCRETE_COM_PROCESS_MODEL_ELEMENTS = true;

   private final RigidBody orientationEstimationLink;

   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;

   private final DenseMatrix64F angularAccelerationNoiseCovariance;
   private final DenseMatrix64F comAccelerationNoiseCovariance;

   private final List<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();
   private final List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();
   private final List<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();

   public ComposableOrientationAndCoMEstimatorCreator(DenseMatrix64F angularAccelerationNoiseCovariance, DenseMatrix64F comAccelerationNoiseCovariance,
         RigidBody orientationEstimationLink, ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort)
   {
      this.angularAccelerationNoiseCovariance = angularAccelerationNoiseCovariance;
      this.comAccelerationNoiseCovariance = comAccelerationNoiseCovariance;
      this.orientationEstimationLink = orientationEstimationLink;
      this.inverseDynamicsStructureOutputPort = inverseDynamicsStructureOutputPort;
   }

   public void addOrientationSensorConfigurations(Collection<OrientationSensorConfiguration> orientationSensorConfigurations)
   {
      for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
      {
         this.addOrientationSensorConfiguration(orientationSensorConfiguration);
      }
   }

   public void addOrientationSensorConfiguration(OrientationSensorConfiguration orientationSensorConfiguration)
   {
      orientationSensorConfigurations.add(orientationSensorConfiguration);
   }

   public void addAngularVelocitySensorConfigurations(Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations)
   {
      for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
      {
         addAngularVelocitySensorConfiguration(angularVelocitySensorConfiguration);
      }
   }

   public void addAngularVelocitySensorConfiguration(AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
   {
      this.angularVelocitySensorConfigurations.add(angularVelocitySensorConfiguration);
   }

   public void addLinearAccelerationSensorConfigurations(Collection<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations)
   {
      for (LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration : linearAccelerationSensorConfigurations)
      {
         this.addLinearAccelerationSensorConfiguration(linearAccelerationSensorConfiguration);
      }
   }

   public void addLinearAccelerationSensorConfiguration(LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration)
   {
      this.linearAccelerationSensorConfigurations.add(linearAccelerationSensorConfiguration);
   }

   public ComposableOrientationAndCoMEstimator createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, ReferenceFrame estimationFrame,
         AfterJointReferenceFrameNameMap estimatorFrameMap, RigidBodyToIndexMap estimatorRigidBodyToIndexMap, YoVariableRegistry registry)
   {
      return new ComposableOrientationAndCoMEstimator("orientationEstimator", controlDT, estimationFrame, estimatorFrameMap, estimatorRigidBodyToIndexMap,
            controlFlowGraph, inverseDynamicsStructureOutputPort, registry);
   }

   //   private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   //   {
   //      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
   //      CommonOps.setIdentity(orientationCovarianceMatrix);
   //      CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);
   //
   //      return orientationCovarianceMatrix;
   //   }

   private static DenseMatrix64F createCovarianceMatrix(double standardDeviationXY, double standardDeviationZ, int size)
   {
      DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);

      orientationCovarianceMatrix.set(0, 0, standardDeviationXY * standardDeviationXY);
      orientationCovarianceMatrix.set(1, 1, standardDeviationXY * standardDeviationXY);
      orientationCovarianceMatrix.set(2, 2, standardDeviationZ * standardDeviationZ);

      return orientationCovarianceMatrix;
   }

   public class ComposableOrientationAndCoMEstimator extends ComposableStateEstimator implements StateEstimatorWithPorts
   {
      private final ControlFlowGraph controlFlowGraph;

      private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
      private final ControlFlowInputPort<FrameVector> desiredCenterOfMassAccelerationInputPort;
      private final ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort;
      private final ControlFlowInputPort<Set<PointPositionDataObject>> pointPositionInputPort;
      private final ControlFlowInputPort<Set<PointVelocityDataObject>> pointVelocityInputPort;

      private final ControlFlowOutputPort<FrameOrientation> orientationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityStatePort;
      private final ControlFlowOutputPort<FramePoint> centerOfMassPositionStatePort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityStatePort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularAccelerationStatePort;

      private final ControlFlowOutputPort<FullInverseDynamicsStructure> updatedInverseDynamicsStructureOutputPort;
      private final OrientationStateFullRobotModelUpdater orientationStateFullRobotModelUpdater;
      private final CenterOfMassBasedFullRobotModelUpdater centerOfMassBasedFullRobotModelUpdater;
      private final CenterOfMassStateFullRobotModelUpdater centerOfMassStateFullRobotModelUpdater;
      private final ReferenceFrame estimationFrame;
      private final boolean originalStateEstimator = true;

      public ComposableOrientationAndCoMEstimator(String name, double controlDT, ReferenceFrame estimationFrame,
            AfterJointReferenceFrameNameMap estimatorFrameMap, RigidBodyToIndexMap estimatorRigidBodyToIndexMap, ControlFlowGraph controlFlowGraph,
            ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort, YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         this.estimationFrame = estimationFrame;
         this.controlFlowGraph = controlFlowGraph;
         this.inverseDynamicsStructureInputPort = createInputPort("inverseDynamicsStructureInputPort");
         controlFlowGraph.connectElements(inverseDynamicsStructureOutputPort, inverseDynamicsStructureInputPort);

         inverseDynamicsStructureInputPort.setData(inverseDynamicsStructureOutputPort.getData());

         orientationStatePort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         angularVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);
         angularAccelerationStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "AngularAcceleration", estimationFrame, registry);
         centerOfMassPositionStatePort = new YoFramePointControlFlowOutputPort(this, name + "CoMPosition", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMVelocity", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassAccelerationStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMAcceleration", ReferenceFrame.getWorldFrame(), registry);

         this.updatedInverseDynamicsStructureOutputPort = createOutputPort("updatedInverseDynamicsStructureOutputPort");

         desiredAngularAccelerationInputPort = createInputPort("desiredAngularAccelerationInputPort");
         desiredCenterOfMassAccelerationInputPort = createInputPort("desiredCenterOfMassAccelerationInputPort");

         pointPositionInputPort = createInputPort("pointPositionInputPort");
         pointPositionInputPort.setData(new LinkedHashSet<PointPositionDataObject>());

         pointVelocityInputPort = createInputPort("pointVelocityInputPort");
         pointVelocityInputPort.setData(new LinkedHashSet<PointVelocityDataObject>());

         addOrientationProcessModelElement();
         addAngularVelocityProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);
         addAngularAccelerationProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);

         addCoMPositionProcessModelElement(controlDT);
         addCoMVelocityProcessModelElement(controlDT, desiredCenterOfMassAccelerationInputPort);
         addCoMAccelerationProcessModelElement(desiredCenterOfMassAccelerationInputPort);

         for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
         {
            addOrientationSensor(estimationFrame, controlFlowGraph, orientationSensorConfiguration);
         }

         for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
         {
            addAngularVelocitySensor(estimationFrame, controlFlowGraph, angularVelocitySensorConfiguration);
         }

         for (LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration : linearAccelerationSensorConfigurations)
         {
            addLinearAccelerationSensor(estimationFrame, controlFlowGraph, linearAccelerationSensorConfiguration);
         }

         addAggregatedPointPositionMeasurementModelElement(pointPositionInputPort, estimationFrame, estimatorFrameMap);
         addAggregatedPointVelocityMeasurementModelElement(pointVelocityInputPort, estimationFrame, estimatorFrameMap, estimatorRigidBodyToIndexMap);

         this.orientationStateFullRobotModelUpdater = new OrientationStateFullRobotModelUpdater(inverseDynamicsStructureInputPort,
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort, orientationStatePort, angularVelocityStatePort,
               angularAccelerationStatePort);

         if (originalStateEstimator)
         {
            this.centerOfMassStateFullRobotModelUpdater = null;
            this.centerOfMassBasedFullRobotModelUpdater = new CenterOfMassBasedFullRobotModelUpdater(inverseDynamicsStructureInputPort,
                  centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort, orientationStatePort,
                  angularVelocityStatePort, angularAccelerationStatePort);
         }
         else
         {
            this.centerOfMassStateFullRobotModelUpdater = new CenterOfMassStateFullRobotModelUpdater(inverseDynamicsStructureInputPort,
                  centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort, orientationStatePort,
                  angularVelocityStatePort, angularAccelerationStatePort);
            this.centerOfMassBasedFullRobotModelUpdater = null;
         }

         Runnable runnable = new Runnable()
         {
            public void run()
            {
               if (originalStateEstimator)
               {
                  centerOfMassBasedFullRobotModelUpdater.run();
                  updatedInverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
               }
               else
               {
                  // TODO: Less magic at a distance communication here.
                  orientationStateFullRobotModelUpdater.run();
                  updatedInverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
                  centerOfMassStateFullRobotModelUpdater.run();
                  updatedInverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
               }

            }
         };

         addPostStateChangeRunnable(runnable);
         //         initialize();
      }

      private void addOrientationProcessModelElement()
      {
         ProcessModelElement processModelElement = new OrientationProcessModelElement(angularVelocityStatePort, orientationStatePort, "orientation", registry);
         addProcessModelElement(orientationStatePort, processModelElement);
      }

      private void addAngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
      {
         AngularVelocityProcessModelElement processModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityStatePort,
               angularAccelerationInputPort, "angularVelocity", registry);

         processModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityStatePort, processModelElement);
      }

      private void addAngularAccelerationProcessModelElement(ReferenceFrame estimationFrame, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
      {
         AngularAccelerationProcessModelElement processModelElement = new AngularAccelerationProcessModelElement("angularAcceleration", estimationFrame,
               registry, angularAccelerationStatePort, angularAccelerationInputPort);
         processModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularAccelerationStatePort, processModelElement);
      }

      private void addCoMPositionProcessModelElement(double controlDT)
      {
         AbstractProcessModelElement processModelElement;

         if (USE_DISCRETE_COM_PROCESS_MODEL_ELEMENTS)
         {
            processModelElement = new CenterOfMassPositionDiscreteProcessModelElement(controlDT, centerOfMassPositionStatePort, centerOfMassVelocityStatePort,
                  "CoMPosition", registry);
         }
         else
         {
            processModelElement = new CenterOfMassPositionProcessModelElement(centerOfMassPositionStatePort, centerOfMassVelocityStatePort, "CoMPosition",
                  registry);
         }
         addProcessModelElement(centerOfMassPositionStatePort, processModelElement);
      }

      private void addCoMVelocityProcessModelElement(double controlDT, ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         AbstractProcessModelElement processModelElement;

         if (USE_DISCRETE_COM_PROCESS_MODEL_ELEMENTS)
         {
            processModelElement = new CenterOfMassVelocityDiscreteProcessModelElement(controlDT, centerOfMassVelocityStatePort,
                  centerOfMassAccelerationStatePort, "CoMVelocity", registry);
         }
         else
         {
            processModelElement = new CenterOfMassVelocityProcessModelElement(centerOfMassVelocityStatePort, centerOfMassAccelerationInputPort, "CoMVelocity",
                  registry);
            processModelElement.setProcessNoiseCovarianceBlock(comAccelerationNoiseCovariance);
         }

         addProcessModelElement(centerOfMassVelocityStatePort, processModelElement);
      }

      private void addCoMAccelerationProcessModelElement(ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         CenterOfMassAccelerationProcessModelElement processModelElement = new CenterOfMassAccelerationProcessModelElement("CoMAcceleration", registry,
               centerOfMassAccelerationStatePort, centerOfMassAccelerationInputPort);
         processModelElement.setProcessNoiseCovarianceBlock(comAccelerationNoiseCovariance);
         addProcessModelElement(centerOfMassAccelerationStatePort, processModelElement);
      }

      private void addOrientationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            OrientationSensorConfiguration orientationSensorConfiguration)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<Matrix3d> measurementInputPort = createInputPort("orientationMeasurementInputPort");

         ControlFlowInputPort<Matrix3d> orientationMeasurementPort = measurementInputPort;
         String name = orientationSensorConfiguration.getName();
         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationStatePort,
               orientationMeasurementPort, estimationFrame, measurementFrame, name, registry);
         orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);

         addMeasurementModelElement(orientationMeasurementModel);
         controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementPort);
      }

      private void addAngularVelocitySensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
      {
         String biasName = angularVelocitySensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
         ControlFlowInputPort<Vector3d> measurementInputPort = createInputPort("angularVelocityMeasurementInputPort");

         ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = measurementInputPort;

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityStatePort,
               biasPort, angularVelocityMeasurementPort, orientationEstimationLink, estimationFrame, measurementLink, measurementFrame,
               inverseDynamicsStructureInputPort, name, registry);
         angularVelocityMeasurementModel.setNoiseCovariance(angularVelocityNoiseCovariance);

         addMeasurementModelElement(angularVelocityMeasurementModel);
         controlFlowGraph.connectElements(angularVelocitySensorConfiguration.getOutputPort(), angularVelocityMeasurementPort);
      }

      private void addLinearAccelerationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration)
      {
         String biasName = linearAccelerationSensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = linearAccelerationSensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = linearAccelerationSensorConfiguration.getLinearAccelerationMeasurementLink();
         ControlFlowInputPort<Vector3d> measurementInputPort = createInputPort("linearAccelerationMeasurementInputPort");

         ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort = measurementInputPort;

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);

         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = linearAccelerationSensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = linearAccelerationSensorConfiguration.getName();

         DenseMatrix64F linearAccelerationNoiseCovariance = linearAccelerationSensorConfiguration.getLinearAccelerationNoiseCovariance();

         double gZ = linearAccelerationSensorConfiguration.getGravityZ();

         LinearAccelerationMeasurementModelElement linearAccelerationMeasurementModel = new LinearAccelerationMeasurementModelElement(name, registry,
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort, orientationStatePort, angularVelocityStatePort,
               angularAccelerationStatePort, biasPort, linearAccelerationMeasurementInputPort, inverseDynamicsStructureInputPort, measurementLink,
               measurementFrame, orientationEstimationLink, estimationFrame, gZ);

         linearAccelerationMeasurementModel.setNoiseCovariance(linearAccelerationNoiseCovariance);

         addMeasurementModelElement(linearAccelerationMeasurementModel);
         controlFlowGraph.connectElements(linearAccelerationSensorConfiguration.getOutputPort(), linearAccelerationMeasurementInputPort);
      }

      private void addAggregatedPointPositionMeasurementModelElement(ControlFlowInputPort<Set<PointPositionDataObject>> pointPositionInputPort,
            ReferenceFrame estimationFrame, AfterJointReferenceFrameNameMap estimatorFrameMap)
      {
         AggregatePointPositionMeasurementModelElement element = new AggregatePointPositionMeasurementModelElement(pointPositionInputPort,
               centerOfMassPositionStatePort, orientationStatePort, estimationFrame, estimatorFrameMap);

         //         DenseMatrix64F covariance = createDiagonalCovarianceMatrix(pointPositionMeasurementStandardDeviation, 3);
         DenseMatrix64F covariance = createCovarianceMatrix(pointPositionXYMeasurementStandardDeviation, pointPositionZMeasurementStandardDeviation, 3);

         element.setNoiseCovariance(covariance);
         addMeasurementModelElement(element);
      }

      private void addAggregatedPointVelocityMeasurementModelElement(ControlFlowInputPort<Set<PointVelocityDataObject>> pointVelocityInputPort,
            ReferenceFrame estimationFrame, AfterJointReferenceFrameNameMap estimatorFrameMap, RigidBodyToIndexMap rigidBodyToIndexMap)
      {
         AggregatePointVelocityMeasurementModelElement element = new AggregatePointVelocityMeasurementModelElement(pointVelocityInputPort,
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, orientationStatePort, angularVelocityStatePort, inverseDynamicsStructureInputPort,
               estimatorFrameMap, rigidBodyToIndexMap, estimationFrame);

         //         DenseMatrix64F covariance = createDiagonalCovarianceMatrix(pointVelocityMeasurementStandardDeviation, 3);
         DenseMatrix64F covariance = createCovarianceMatrix(pointVelocityXYMeasurementStandardDeviation, pointVelocityZMeasurementStandardDeviation, 3);

         element.setNoiseCovariance(covariance);
         addMeasurementModelElement(element);
      }

      public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
      {
         estimatedOrientationToPack.setAndChangeFrame(orientationStatePort.getData());
      }

      public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
      {
         estimatedAngularVelocityToPack.setAndChangeFrame(angularVelocityStatePort.getData());
      }

      public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
      {
         estimatedCoMPositionToPack.setAndChangeFrame(centerOfMassPositionStatePort.getData());
      }

      public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
      {
         estimatedCoMVelocityToPack.setAndChangeFrame(centerOfMassVelocityStatePort.getData());
      }

      public void setEstimatedOrientation(FrameOrientation orientation)
      {
         orientationStatePort.setData(orientation);
         if (originalStateEstimator)
         {
            centerOfMassBasedFullRobotModelUpdater.run();
         }
         else
         {
            centerOfMassStateFullRobotModelUpdater.run();
         }
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void setEstimatedAngularVelocity(FrameVector angularVelocity)
      {
         angularVelocityStatePort.setData(angularVelocity);
         if (originalStateEstimator)
         {
            centerOfMassBasedFullRobotModelUpdater.run();
         }
         else
         {
            centerOfMassStateFullRobotModelUpdater.run();
         }
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
      {
         centerOfMassPositionStatePort.setData(estimatedCoMPosition);
         if (originalStateEstimator)
         {
            centerOfMassBasedFullRobotModelUpdater.run();
         }
         else
         {
            centerOfMassStateFullRobotModelUpdater.run();
         }
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
      {
         centerOfMassVelocityStatePort.setData(estimatedCoMVelocity);
         if (originalStateEstimator)
         {
            centerOfMassBasedFullRobotModelUpdater.run();
         }
         else
         {
            centerOfMassStateFullRobotModelUpdater.run();
         }
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void initializeOrientationEstimateToMeasurement()
      {
         if (orientationSensorConfigurations.size() > 0)
         {
            // R^W_M
            OrientationSensorConfiguration firstOrientationSensorConfiguration = orientationSensorConfigurations.get(0);
            ControlFlowOutputPort<Matrix3d> firstOrientationMeasurementOutputPort = firstOrientationSensorConfiguration.getOutputPort();
            Matrix3d measurementToWorld = firstOrientationMeasurementOutputPort.getData();

            // R^M_E
            ReferenceFrame measurementFrame = firstOrientationSensorConfiguration.getMeasurementFrame();
            FrameOrientation estimationFrameOrientation = new FrameOrientation(estimationFrame);
            estimationFrameOrientation.changeFrame(measurementFrame);
            Matrix3d estimationToMeasurement = estimationFrameOrientation.getMatrix3d();

            // R^W_E
            Matrix3d estimationToWorld = new Matrix3d();
            estimationToWorld.mul(measurementToWorld, estimationToMeasurement);
            FrameOrientation initialOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), estimationToWorld);
            setEstimatedOrientation(initialOrientation);
         }
      }

      private void setUpdatedInverseDynamicsStructureOutputPort(FullInverseDynamicsStructure inverseDynamicsStructure)
      {
         updatedInverseDynamicsStructureOutputPort.setData(inverseDynamicsStructure);
      }

      public DenseMatrix64F getCovariance()
      {
         return kalmanFilter.getCovariance();
      }

      public DenseMatrix64F getState()
      {
         return kalmanFilter.getState();
      }

      public void setState(DenseMatrix64F x, DenseMatrix64F covariance)
      {
         kalmanFilter.setState(x, covariance);
      }

      public ControlFlowInputPort<FrameVector> getDesiredAngularAccelerationInputPort()
      {
         return desiredAngularAccelerationInputPort;
      }

      public ControlFlowInputPort<FrameVector> getDesiredCenterOfMassAccelerationInputPort()
      {
         return desiredCenterOfMassAccelerationInputPort;
      }

      public ControlFlowInputPort<Set<PointPositionDataObject>> getPointPositionInputPort()
      {
         return pointPositionInputPort;
      }

      public ControlFlowInputPort<Set<PointVelocityDataObject>> getPointVelocityInputPort()
      {
         return pointVelocityInputPort;
      }

      public ControlFlowGraph getControlFlowGraph()
      {
         return controlFlowGraph;
      }

   }
}
