package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFramePointControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.AngularVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.LinearAccelerationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.OrientationMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements.PointVelocityMeasurementModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AngularAccelerationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.AngularVelocityProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.BiasProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassAccelerationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassPositionProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.CenterOfMassVelocityProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.OrientationProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.processModelElements.ProcessModelElement;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocitySensorConfiguration;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableOrientationAndCoMEstimatorCreator
{
   private static final int VECTOR3D_LENGTH = 3;

   private final RigidBody orientationEstimationLink;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final DenseMatrix64F angularAccelerationNoiseCovariance;
   private final DenseMatrix64F comAccelerationNoiseCovariance;

   private final List<OrientationSensorConfiguration> orientationSensorConfigurations = new ArrayList<OrientationSensorConfiguration>();
   private final List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations = new ArrayList<AngularVelocitySensorConfiguration>();
   private final List<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations = new ArrayList<LinearAccelerationSensorConfiguration>();
   private final List<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations = new ArrayList<PointVelocitySensorConfiguration>();

   public ComposableOrientationAndCoMEstimatorCreator(DenseMatrix64F angularAccelerationNoiseCovariance, DenseMatrix64F comAccelerationNoiseCovariance,
         RigidBody orientationEstimationLink, TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.angularAccelerationNoiseCovariance = angularAccelerationNoiseCovariance;
      this.comAccelerationNoiseCovariance = comAccelerationNoiseCovariance;
      this.orientationEstimationLink = orientationEstimationLink;
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
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

   public void addPointVelocitySensorConfigurations(Collection<PointVelocitySensorConfiguration> pointVelocitySensorConfigurations)
   {
      for (PointVelocitySensorConfiguration pointVelocitySensorConfiguration : pointVelocitySensorConfigurations)
      {
         this.addPointVelocitySensorConfiguration(pointVelocitySensorConfiguration);
      }
   }
   
   public void addPointVelocitySensorConfiguration(PointVelocitySensorConfiguration pointVelocitySensorConfiguration)
   {
      this.pointVelocitySensorConfigurations.add(pointVelocitySensorConfiguration);      
   }
   
   public OrientationEstimator createOrientationEstimator(ControlFlowGraph controlFlowGraph, double controlDT, SixDoFJoint rootJoint, RigidBody estimationLink,
         ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort,
         ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort, YoVariableRegistry registry)
   {
      return new ComposableOrientationAndCoMEstimator("orientationEstimator", controlDT, rootJoint, estimationLink, estimationFrame, controlFlowGraph,
            desiredAngularAccelerationOutputPort, desiredCenterOfMassAccelerationOutputPort, registry);
   }

   private class ComposableOrientationAndCoMEstimator extends ComposableStateEstimator implements OrientationEstimator
   {
      private final ControlFlowOutputPort<FrameOrientation> orientationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularVelocityStatePort;
      private final ControlFlowOutputPort<FramePoint> centerOfMassPositionStatePort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityStatePort;
      private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationStatePort;
      private final ControlFlowOutputPort<FrameVector> angularAccelerationStatePort;

      public ComposableOrientationAndCoMEstimator(String name, double controlDT, SixDoFJoint rootJoint, RigidBody estimationLink,
            ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph, ControlFlowOutputPort<FrameVector> desiredAngularAccelerationOutputPort,
            ControlFlowOutputPort<FrameVector> desiredCenterOfMassAccelerationOutputPort, YoVariableRegistry parentRegistry)
      {
         super(name, controlDT, parentRegistry);

         orientationStatePort = new YoFrameQuaternionControlFlowOutputPort(this, name, ReferenceFrame.getWorldFrame(), parentRegistry);
         angularVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "Omega", estimationFrame, registry);
         angularAccelerationStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "AngularAcceleration", estimationFrame, registry);
         centerOfMassPositionStatePort = new YoFramePointControlFlowOutputPort(this, name + "CoMPosition", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassVelocityStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMVelocity", ReferenceFrame.getWorldFrame(), registry);
         centerOfMassAccelerationStatePort = new YoFrameVectorControlFlowOutputPort(this, name + "CoMAcceleration", ReferenceFrame.getWorldFrame(), registry);

         ControlFlowInputPort<FrameVector> desiredAngularAccelerationInputPort = createProcessInputPort(VECTOR3D_LENGTH);
         controlFlowGraph.connectElements(desiredAngularAccelerationOutputPort, desiredAngularAccelerationInputPort);

         ControlFlowInputPort<FrameVector> desiredCenterOfMassAccelerationInputPort = createProcessInputPort(VECTOR3D_LENGTH);
         controlFlowGraph.connectElements(desiredCenterOfMassAccelerationOutputPort, desiredCenterOfMassAccelerationInputPort);

         addOrientationProcessModelElement();
         addAngularVelocityProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);
         addAngularAccelerationProcessModelElement(estimationFrame, desiredAngularAccelerationInputPort);

         addCoMPositionProcessModelElement();
         addCoMVelocityProcessModelElement(desiredCenterOfMassAccelerationInputPort);
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
         
         for (PointVelocitySensorConfiguration pointVelocitySensorConfiguration : pointVelocitySensorConfigurations)
         {
            addPointVelocitySensor(estimationFrame, controlFlowGraph, pointVelocitySensorConfiguration);
         }

         CenterOfMassBasedFullRobotModelUpdater centerOfMassBasedFullRobotModelUpdater = new CenterOfMassBasedFullRobotModelUpdater(twistCalculator,
               spatialAccelerationCalculator, centerOfMassPositionStatePort, centerOfMassVelocityStatePort, centerOfMassAccelerationStatePort,
               orientationStatePort, angularVelocityStatePort, angularAccelerationStatePort, estimationLink, estimationFrame, rootJoint);

         addPostStateChangeRunnable(centerOfMassBasedFullRobotModelUpdater);

         initialize();
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

      private void addCoMPositionProcessModelElement()
      {
         CenterOfMassPositionProcessModelElement processModelElement = new CenterOfMassPositionProcessModelElement(centerOfMassPositionStatePort,
               centerOfMassVelocityStatePort, "CoMPosition", registry);
         addProcessModelElement(centerOfMassPositionStatePort, processModelElement);
      }

      private void addCoMVelocityProcessModelElement(ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
      {
         CenterOfMassVelocityProcessModelElement processModelElement = new CenterOfMassVelocityProcessModelElement(centerOfMassVelocityStatePort,
               centerOfMassAccelerationInputPort, "CoMVelocity", registry);
         processModelElement.setProcessNoiseCovarianceBlock(comAccelerationNoiseCovariance);
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
         ControlFlowInputPort<Matrix3d> orientationMeasurementPort = createMeasurementInputPort(VECTOR3D_LENGTH);
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
         ControlFlowInputPort<Vector3d> angularVelocityMeasurementPort = createMeasurementInputPort(VECTOR3D_LENGTH);

         ControlFlowOutputPort<FrameVector> biasPort = new YoFrameVectorControlFlowOutputPort(this, biasName, measurementFrame, registry);
         BiasProcessModelElement biasProcessModelElement = new BiasProcessModelElement(biasPort, measurementFrame, biasName, registry);
         DenseMatrix64F biasProcessNoiseCovariance = angularVelocitySensorConfiguration.getBiasProcessNoiseCovariance();
         biasProcessModelElement.setProcessNoiseCovarianceBlock(biasProcessNoiseCovariance);
         addProcessModelElement(biasPort, biasProcessModelElement);
         String name = angularVelocitySensorConfiguration.getName();
         DenseMatrix64F angularVelocityNoiseCovariance = angularVelocitySensorConfiguration.getAngularVelocityNoiseCovariance();

         AngularVelocityMeasurementModelElement angularVelocityMeasurementModel = new AngularVelocityMeasurementModelElement(angularVelocityStatePort,
               biasPort, angularVelocityMeasurementPort, orientationEstimationLink, estimationFrame, measurementLink, measurementFrame, twistCalculator, name,
               registry);
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
         ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort = createMeasurementInputPort(VECTOR3D_LENGTH);

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
               angularAccelerationStatePort, biasPort, linearAccelerationMeasurementInputPort, twistCalculator, spatialAccelerationCalculator, measurementLink,
               measurementFrame, orientationEstimationLink, estimationFrame, gZ);

         linearAccelerationMeasurementModel.setNoiseCovariance(linearAccelerationNoiseCovariance);

         addMeasurementModelElement(linearAccelerationMeasurementModel);
         controlFlowGraph.connectElements(linearAccelerationSensorConfiguration.getOutputPort(), linearAccelerationMeasurementInputPort);
      }
      
      private void addPointVelocitySensor(ReferenceFrame estimationFrame, ControlFlowGraph controlFlowGraph,
            PointVelocitySensorConfiguration pointVelocitySensorConfiguration)
      {
         RigidBody measurementLink = pointVelocitySensorConfiguration.getPointVelocityMeasurementLink();
         FramePoint stationaryPoint = pointVelocitySensorConfiguration.getPointVelocityMeasurementPoint();

         ControlFlowInputPort<Vector3d> pointVelocityMeasurementInputPort = createMeasurementInputPort(VECTOR3D_LENGTH);

         String name = pointVelocitySensorConfiguration.getName();

         DenseMatrix64F pointVelocityNoiseCovariance = pointVelocitySensorConfiguration.getPointVelocityNoiseCovariance();


         PointVelocityMeasurementModelElement pointVelocityMeasurementModelElement = new PointVelocityMeasurementModelElement(
               name, pointVelocityMeasurementInputPort, 
               centerOfMassPositionStatePort, centerOfMassVelocityStatePort, orientationStatePort, 
               angularVelocityStatePort, estimationFrame, 
               measurementLink, stationaryPoint, twistCalculator, registry);

         pointVelocityMeasurementModelElement.setNoiseCovariance(pointVelocityNoiseCovariance);

         addMeasurementModelElement(pointVelocityMeasurementModelElement);
         controlFlowGraph.connectElements(pointVelocitySensorConfiguration.getOutputPort(), pointVelocityMeasurementInputPort);
      }

      public FrameOrientation getEstimatedOrientation()
      {
         return orientationStatePort.getData();
      }

      public FrameVector getEstimatedAngularVelocity()
      {
         return angularVelocityStatePort.getData();
      }

      public FramePoint getEstimatedCoMPosition()
      {
         return centerOfMassPositionStatePort.getData();
      }
      
      public FrameVector getEstimatedCoMVelocity()
      {
         return centerOfMassVelocityStatePort.getData();
      }

      public void setEstimatedOrientation(FrameOrientation orientation)
      {
         orientationStatePort.setData(orientation);
      }

      public void setEstimatedAngularVelocity(FrameVector angularVelocity)
      {
         angularVelocityStatePort.setData(angularVelocity);
      }
      
      public void setEstimatedCoMPosition(FramePoint estimatedCoMPosition)
      {
         centerOfMassPositionStatePort.setData(estimatedCoMPosition);
      }
      
      public void setEstimatedCoMVelocity(FrameVector estimatedCoMVelocity)
      {
         centerOfMassVelocityStatePort.setData(estimatedCoMVelocity);
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

   }
}
