package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFramePointControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.AggregatePointPositionMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.AggregatePointVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.LinearAccelerationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.OrientationMeasurementModelElement;
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
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ComposableOrientationAndCoMEstimatorCreator
{
   private static final boolean USE_DISCRETE_COM_PROCESS_MODEL_ELEMENTS = true;

   private final RigidBody orientationEstimationLink;

   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;

   private final PointMeasurementNoiseParameters pointMeasurementNoiseParameters;
   private final DenseMatrix64F angularAccelerationNoiseCovariance;
   private final DenseMatrix64F comAccelerationNoiseCovariance;

   private final List<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();
   private final List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();
   private final List<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();
   private final boolean assumePerfectIMU;

   public ComposableOrientationAndCoMEstimatorCreator(PointMeasurementNoiseParameters pointMeasurementNoiseParameters,
         DenseMatrix64F angularAccelerationNoiseCovariance, DenseMatrix64F comAccelerationNoiseCovariance,
           RigidBody orientationEstimationLink, ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort,
           boolean assumePerfectIMU)
   {
      this.pointMeasurementNoiseParameters = pointMeasurementNoiseParameters;
      this.angularAccelerationNoiseCovariance = angularAccelerationNoiseCovariance;
      this.comAccelerationNoiseCovariance = comAccelerationNoiseCovariance;
      this.orientationEstimationLink = orientationEstimationLink;
      this.inverseDynamicsStructureOutputPort = inverseDynamicsStructureOutputPort;
      this.assumePerfectIMU = assumePerfectIMU;
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

   public ComposableOrientationAndCoMEstimator createOrientationAndCoMEstimator(ControlFlowGraph controlFlowGraph, double controlDT,
           ReferenceFrame estimationFrame, AfterJointReferenceFrameNameMap estimatorFrameMap, RigidBodyToIndexMap estimatorRigidBodyToIndexMap,
           YoVariableRegistry registry)
   {
      return new ComposableOrientationAndCoMEstimator("orientationAndCoMEstimator", controlDT, estimationFrame, estimatorFrameMap,
              estimatorRigidBodyToIndexMap, controlFlowGraph, inverseDynamicsStructureOutputPort, registry, assumePerfectIMU);
   }

   // private static DenseMatrix64F createDiagonalCovarianceMatrix(double standardDeviation, int size)
   // {
   // DenseMatrix64F orientationCovarianceMatrix = new DenseMatrix64F(size, size);
   // CommonOps.setIdentity(orientationCovarianceMatrix);
   // CommonOps.scale(MathTools.square(standardDeviation), orientationCovarianceMatrix);
   //
   // return orientationCovarianceMatrix;
   // }

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
      private final ControlFlowInputPort<List<PointPositionDataObject>> pointPositionInputPort;
      private final ControlFlowInputPort<List<PointVelocityDataObject>> pointVelocityInputPort;

      private final ControlFlowOutputPort<FrameOrientation> orientationOutputPort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityOutputPort;

      private final ControlFlowOutputPort<FramePoint> centerOfMassPositionOutputPort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityOutputPort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationOutputPort;
      private final ControlFlowOutputPort<FrameVector> angularAccelerationOutputPort;

      private final ControlFlowOutputPort<FullInverseDynamicsStructure> updatedInverseDynamicsStructureOutputPort;
      private final OrientationStateRobotModelUpdater orientationStateRobotModelUpdater;

      // private final OrientationAndPositionFullRobotModelUpdater orientationAndPositionFullRobotModelUpdater;
      private final PositionStateRobotModelUpdater positionStateRobotModelUpdater;
      private final ReferenceFrame estimationFrame;
      private final boolean assumePerfectIMU;

      public ComposableOrientationAndCoMEstimator(String name, double controlDT, ReferenceFrame estimationFrame,
              AfterJointReferenceFrameNameMap estimatorFrameMap, RigidBodyToIndexMap estimatorRigidBodyToIndexMap, ControlFlowGraph controlFlowGraph,
              ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort, 
              YoVariableRegistry parentRegistry,
              final boolean assumePerfectIMU)
      {
         super(name, controlDT, parentRegistry);
         this.assumePerfectIMU = assumePerfectIMU;
         this.estimationFrame = estimationFrame;
         this.controlFlowGraph = controlFlowGraph;
         this.inverseDynamicsStructureInputPort = createInputPort("inverseDynamicsStructureInputPort");
         controlFlowGraph.connectElements(inverseDynamicsStructureOutputPort, inverseDynamicsStructureInputPort);

         inverseDynamicsStructureInputPort.setData(inverseDynamicsStructureOutputPort.getData());
         desiredAngularAccelerationInputPort = createInputPort("desiredAngularAccelerationInputPort");
         desiredCenterOfMassAccelerationInputPort = createInputPort("desiredCenterOfMassAccelerationInputPort");
         pointPositionInputPort = createInputPort("pointPositionInputPort");
         pointPositionInputPort.setData(new ArrayList<PointPositionDataObject>());
         pointVelocityInputPort = createInputPort("pointVelocityInputPort");
         pointVelocityInputPort.setData(new ArrayList<PointVelocityDataObject>());

         this.updatedInverseDynamicsStructureOutputPort = createOutputPort("updatedInverseDynamicsStructureOutputPort");
         centerOfMassPositionOutputPort = new YoFramePointControlFlowOutputPort(this, name + "CoMPosition", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassVelocityOutputPort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMVelocity", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassAccelerationOutputPort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMAcceleration", ReferenceFrame.getWorldFrame(), registry);

         if (!assumePerfectIMU)
         {
            orientationOutputPort = new YoFrameQuaternionControlFlowOutputPort(this, name + "Orientation", ReferenceFrame.getWorldFrame(), registry);
            angularVelocityOutputPort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);
            angularAccelerationOutputPort = new YoFrameVectorControlFlowOutputPort(this, name + "AngularAcceleration", estimationFrame, registry);

            addOrientationProcessModelElement();
            addAngularVelocityProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);
            addAngularAccelerationProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);

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

            this.orientationStateRobotModelUpdater = new OrientationStateRobotModelUpdater(getInverseDynamicsStructureInputPort(), getOrientationOutputPort(),
                    getAngularVelocityOutputPort());
            this.positionStateRobotModelUpdater = new PositionStateRobotModelUpdater(getInverseDynamicsStructureInputPort(), centerOfMassPositionOutputPort,
                    centerOfMassVelocityOutputPort);
         }
         else
         {
            orientationOutputPort = null;
            angularVelocityOutputPort = null;
            angularAccelerationOutputPort = null;

            this.orientationStateRobotModelUpdater = null;
            this.positionStateRobotModelUpdater = new PositionStateRobotModelUpdater(getInverseDynamicsStructureInputPort(), centerOfMassPositionOutputPort,
                    centerOfMassVelocityOutputPort);
         }

         addCoMPositionProcessModelElement(controlDT);
         addCoMVelocityProcessModelElement(controlDT, desiredCenterOfMassAccelerationInputPort);
         addCoMAccelerationProcessModelElement(desiredCenterOfMassAccelerationInputPort);

         addAggregatedPointPositionMeasurementModelElement(pointPositionInputPort, estimationFrame, estimatorFrameMap, pointMeasurementNoiseParameters);
         addAggregatedPointVelocityMeasurementModelElement(pointVelocityInputPort, estimationFrame, estimatorFrameMap, estimatorRigidBodyToIndexMap, pointMeasurementNoiseParameters);

         Runnable runnable = new Runnable()
         {
            public void run()
            {
               if (!assumePerfectIMU)
               {
                  orientationStateRobotModelUpdater.run();
                  positionStateRobotModelUpdater.run();
               }
               else
               {
                  positionStateRobotModelUpdater.run();
               }
               
               updatedInverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
            }
         };
         addPostStateChangeRunnable(runnable);
      }

      private void addOrientationProcessModelElement()
      {
         ProcessModelElement processModelElement = new OrientationProcessModelElement(angularVelocityOutputPort, orientationOutputPort, "orientation",
                                                      registry);
         addProcessModelElement(orientationOutputPort, processModelElement);
      }

      private void addAngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
      {
         AngularVelocityProcessModelElement processModelElement = new AngularVelocityProcessModelElement(estimationFrame, angularVelocityOutputPort,
                                                                     angularAccelerationInputPort, "angularVelocity", registry);

         processModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularVelocityOutputPort, processModelElement);
      }

      private void addAngularAccelerationProcessModelElement(ReferenceFrame estimationFrame, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
      {
         AngularAccelerationProcessModelElement processModelElement = new AngularAccelerationProcessModelElement("angularAcceleration", estimationFrame,
                                                                         registry, angularAccelerationOutputPort, angularAccelerationInputPort);
         processModelElement.setProcessNoiseCovarianceBlock(angularAccelerationNoiseCovariance);
         addProcessModelElement(angularAccelerationOutputPort, processModelElement);
      }

      private void addCoMPositionProcessModelElement(double controlDT)
      {
         AbstractProcessModelElement processModelElement;

         if (USE_DISCRETE_COM_PROCESS_MODEL_ELEMENTS)
         {
            processModelElement = new CenterOfMassPositionDiscreteProcessModelElement(controlDT, centerOfMassPositionOutputPort,
                    centerOfMassVelocityOutputPort, "CoMPosition", registry);
         }
         else
         {
            processModelElement = new CenterOfMassPositionProcessModelElement(centerOfMassPositionOutputPort, centerOfMassVelocityOutputPort, "CoMPosition",
                    registry);
         }

         addProcessModelElement(centerOfMassPositionOutputPort, processModelElement);
      }

      private void addCoMVelocityProcessModelElement(double controlDT, ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         AbstractProcessModelElement processModelElement;

         if (USE_DISCRETE_COM_PROCESS_MODEL_ELEMENTS)
         {
            processModelElement = new CenterOfMassVelocityDiscreteProcessModelElement(controlDT, centerOfMassVelocityOutputPort,
                    centerOfMassAccelerationOutputPort, "CoMVelocity", registry);
         }
         else
         {
            processModelElement = new CenterOfMassVelocityProcessModelElement(centerOfMassVelocityOutputPort, centerOfMassAccelerationInputPort, "CoMVelocity",
                    registry);
            processModelElement.setProcessNoiseCovarianceBlock(comAccelerationNoiseCovariance);
         }

         addProcessModelElement(centerOfMassVelocityOutputPort, processModelElement);
      }

      private void addCoMAccelerationProcessModelElement(ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         CenterOfMassAccelerationProcessModelElement processModelElement = new CenterOfMassAccelerationProcessModelElement("CoMAcceleration", registry,
                                                                              centerOfMassAccelerationOutputPort, centerOfMassAccelerationInputPort);
         processModelElement.setProcessNoiseCovarianceBlock(comAccelerationNoiseCovariance);
         addProcessModelElement(centerOfMassAccelerationOutputPort, processModelElement);
      }

      private void addOrientationSensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
                                        OrientationSensorConfiguration orientationSensorConfiguration)
      {
         ReferenceFrame measurementFrame = orientationSensorConfiguration.getMeasurementFrame();
         ControlFlowInputPort<RotationMatrix> measurementInputPort = createInputPort("orientationMeasurementInputPort");

         ControlFlowInputPort<RotationMatrix> orientationMeasurementInputPort = measurementInputPort;
         String name = orientationSensorConfiguration.getName();
         DenseMatrix64F orientationNoiseCovariance = orientationSensorConfiguration.getOrientationNoiseCovariance();

         OrientationMeasurementModelElement orientationMeasurementModel = new OrientationMeasurementModelElement(orientationOutputPort,
                                                                             orientationMeasurementInputPort, estimationFrame, measurementFrame, name,
                                                                             registry);
         orientationMeasurementModel.setNoiseCovariance(orientationNoiseCovariance);

         addMeasurementModelElement(orientationMeasurementModel);

         controlFlowGraph.connectElements(orientationSensorConfiguration.getOutputPort(), orientationMeasurementInputPort);
      }

      private void addAngularVelocitySensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
              AngularVelocitySensorConfiguration angularVelocitySensorConfiguration)
      {
         String biasName = angularVelocitySensorConfiguration.getName() + "BiasEstimate";
         ReferenceFrame measurementFrame = angularVelocitySensorConfiguration.getMeasurementFrame();
         RigidBody measurementLink = angularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
         ControlFlowInputPort<Vector3D> measurementInputPort = createInputPort("angularVelocityMeasurementInputPort");

         ControlFlowInputPort<Vector3D> angularVelocityMeasurementPort = measurementInputPort;

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityOutputPort,
                                                                                     biasPort, angularVelocityMeasurementPort, orientationEstimationLink,
                                                                                     estimationFrame, measurementLink, measurementFrame,
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
         ControlFlowInputPort<Vector3D> measurementInputPort = createInputPort("linearAccelerationMeasurementInputPort");

         ControlFlowInputPort<Vector3D> linearAccelerationMeasurementInputPort = measurementInputPort;

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);

         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = linearAccelerationSensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = linearAccelerationSensorConfiguration.getName();

         DenseMatrix64F linearAccelerationNoiseCovariance = linearAccelerationSensorConfiguration.getLinearAccelerationNoiseCovariance();

         double gZ = linearAccelerationSensorConfiguration.getGravityZ();

         LinearAccelerationMeasurementModelElement linearAccelerationMeasurementModel = new LinearAccelerationMeasurementModelElement(name, registry,
                                                                                           centerOfMassPositionOutputPort, centerOfMassVelocityOutputPort,
                                                                                           centerOfMassAccelerationOutputPort, orientationOutputPort,
                                                                                           angularVelocityOutputPort, angularAccelerationOutputPort, biasPort,
                                                                                           linearAccelerationMeasurementInputPort,
                                                                                           inverseDynamicsStructureInputPort, measurementLink,
                                                                                           measurementFrame, orientationEstimationLink, estimationFrame, gZ);

         linearAccelerationMeasurementModel.setNoiseCovariance(linearAccelerationNoiseCovariance);

         addMeasurementModelElement(linearAccelerationMeasurementModel);
         controlFlowGraph.connectElements(linearAccelerationSensorConfiguration.getOutputPort(), linearAccelerationMeasurementInputPort);
      }

      private void addAggregatedPointPositionMeasurementModelElement(ControlFlowInputPort<List<PointPositionDataObject>> pointPositionInputPort,
              ReferenceFrame estimationFrame, AfterJointReferenceFrameNameMap estimatorFrameMap, PointMeasurementNoiseParameters pointMeasurementNoiseParameters)
      {
         AggregatePointPositionMeasurementModelElement element = new AggregatePointPositionMeasurementModelElement(pointPositionInputPort,
                                                                    centerOfMassPositionOutputPort, orientationOutputPort, estimationFrame, estimatorFrameMap,
                                                                    assumePerfectIMU);

         // DenseMatrix64F covariance = createDiagonalCovarianceMatrix(pointPositionMeasurementStandardDeviation, 3);
         DenseMatrix64F covariance = createCovarianceMatrix(pointMeasurementNoiseParameters.getPointPositionXYMeasurementStandardDeviation(), 
               pointMeasurementNoiseParameters.getPointPositionZMeasurementStandardDeviation(), 3);

         element.setNoiseCovariance(covariance);
         addMeasurementModelElement(element);
      }

      private void addAggregatedPointVelocityMeasurementModelElement(ControlFlowInputPort<List<PointVelocityDataObject>> pointVelocityInputPort,
              ReferenceFrame estimationFrame, AfterJointReferenceFrameNameMap estimatorFrameMap, RigidBodyToIndexMap rigidBodyToIndexMap, 
              PointMeasurementNoiseParameters pointMeasurementNoiseParameters)
      {
         AggregatePointVelocityMeasurementModelElement element = new AggregatePointVelocityMeasurementModelElement(pointVelocityInputPort,
                                                                    centerOfMassPositionOutputPort, centerOfMassVelocityOutputPort, orientationOutputPort,
                                                                    angularVelocityOutputPort, inverseDynamicsStructureInputPort, estimatorFrameMap,
                                                                    rigidBodyToIndexMap, estimationFrame, assumePerfectIMU);

         // DenseMatrix64F covariance = createDiagonalCovarianceMatrix(pointVelocityMeasurementStandardDeviation, 3);
         DenseMatrix64F covariance = createCovarianceMatrix(pointMeasurementNoiseParameters.getPointVelocityXYMeasurementStandardDeviation(), 
               pointMeasurementNoiseParameters.getPointVelocityZMeasurementStandardDeviation(), 3);

         element.setNoiseCovariance(covariance);
         addMeasurementModelElement(element);
      }

      public void getEstimatedOrientation(FrameOrientation estimatedOrientationToPack)
      {
         estimatedOrientationToPack.setIncludingFrame(orientationOutputPort.getData());
      }

      public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
      {
         estimatedAngularVelocityToPack.setIncludingFrame(angularVelocityOutputPort.getData());
      }

      public void getEstimatedCoMPosition(FramePoint estimatedCoMPositionToPack)
      {
         estimatedCoMPositionToPack.setIncludingFrame(centerOfMassPositionOutputPort.getData());
      }

      public void getEstimatedCoMVelocity(FrameVector estimatedCoMVelocityToPack)
      {
         estimatedCoMVelocityToPack.setIncludingFrame(centerOfMassVelocityOutputPort.getData());
      }

      public void setEstimatedOrientation(FrameOrientation orientation)
      {
         if (!assumePerfectIMU)
         {
        	 return;
         }
         orientationOutputPort.setData(orientation);
         orientationStateRobotModelUpdater.run();
         positionStateRobotModelUpdater.run();
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void setEstimatedAngularVelocity(FrameVector angularVelocity)
      {
         if (!assumePerfectIMU)
         {
            return;
         }
         angularVelocityOutputPort.setData(angularVelocity);
         orientationStateRobotModelUpdater.run();
         positionStateRobotModelUpdater.run();
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
      {
         centerOfMassPositionOutputPort.setData(estimatedCoMPosition);

         if (!assumePerfectIMU)
         {
            orientationStateRobotModelUpdater.run();
         }

         positionStateRobotModelUpdater.run();
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
      {
         centerOfMassVelocityOutputPort.setData(estimatedCoMVelocity);

         if (!assumePerfectIMU)
         {
            orientationStateRobotModelUpdater.run();
         }

         positionStateRobotModelUpdater.run();
         FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
         inverseDynamicsStructure.updateInternalState();
         setUpdatedInverseDynamicsStructureOutputPort(inverseDynamicsStructure);
      }

      public void initializeOrientationEstimateToMeasurement()
      {
         if ((orientationSensorConfigurations.size() > 0) &&!assumePerfectIMU)
         {
            // R^W_M
            OrientationSensorConfiguration firstOrientationSensorConfiguration = orientationSensorConfigurations.get(0);
            ControlFlowOutputPort<RotationMatrix> firstOrientationMeasurementOutputPort = firstOrientationSensorConfiguration.getOutputPort();
            RotationMatrix measurementToWorld = firstOrientationMeasurementOutputPort.getData();

            // R^M_E
            ReferenceFrame measurementFrame = firstOrientationSensorConfiguration.getMeasurementFrame();
            FrameOrientation estimationFrameOrientation = new FrameOrientation(estimationFrame);
            estimationFrameOrientation.changeFrame(measurementFrame);
            RotationMatrix estimationToMeasurement = estimationFrameOrientation.getMatrix3dCopy();

            // R^W_E
            RotationMatrix estimationToWorld = new RotationMatrix();
            estimationToWorld.set(measurementToWorld);
            estimationToWorld.multiply(estimationToMeasurement);
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

      public ControlFlowInputPort<List<PointPositionDataObject>> getPointPositionInputPort()
      {
         return pointPositionInputPort;
      }

      public ControlFlowInputPort<List<PointVelocityDataObject>> getPointVelocityInputPort()
      {
         return pointVelocityInputPort;
      }

      public ControlFlowGraph getControlFlowGraph()
      {
         return controlFlowGraph;
      }

      public ControlFlowOutputPort<FrameOrientation> getOrientationOutputPort()
      {
         return orientationOutputPort;
      }

      public ControlFlowOutputPort<FrameVector> getAngularVelocityOutputPort()
      {
         return angularVelocityOutputPort;
      }

      public ControlFlowOutputPort<FullInverseDynamicsStructure> getInverseDynamicsStructureOutputPort()
      {
         return updatedInverseDynamicsStructureOutputPort;
      }

      public ControlFlowInputPort<FullInverseDynamicsStructure> getInverseDynamicsStructureInputPort()
      {
         return inverseDynamicsStructureInputPort;
      }

      public void getEstimatedPelvisPosition(FramePoint estimatedPelvisPositionToPack)
      {
         throw new RuntimeException("No pelvis position estimated with this state estimator (" + getClass().getSimpleName() + ").");
      }

      public void getEstimatedPelvisLinearVelocity(FrameVector estimatedPelvisLinearVelocityToPack)
      {
         throw new RuntimeException("No pelvis velocity estimated with this state estimator (" + getClass().getSimpleName() + ").");
      }

   }
}
