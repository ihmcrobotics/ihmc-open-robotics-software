package us.ihmc.sensorProcessing.stateEstimation;

import java.util.Collection;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPort;
import us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;

public class IMUSelectorAndDataConverter extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<RotationMatrix> orientationInputPort;
   private final ControlFlowInputPort<Vector3D> angularVelocityInputPort;
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FrameQuaternion> orientationOutputPort;
   private final ControlFlowOutputPort<FrameVector3D> angularVelocityOutputPort;
   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;

   private final RigidBody estimationLink;
   private final RigidBody angularVelocityMeasurementLink;

   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame orientationMeasurementFrame;
   private final ReferenceFrame angularVelocityMeasurementFrame;
   
   private final YoVariableRegistry registry;
   private final YoDouble imuSimulatedDriftYawAcceleration;
   private final YoDouble imuSimulatedDriftYawVelocity;
   private final YoDouble imuSimulatedDriftYawAngle;
   
   private final double estimatorDT;

   public IMUSelectorAndDataConverter(ControlFlowGraph controlFlowGraph, Collection<OrientationSensorConfiguration> orientationSensorConfigurations,
                                      Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations,
                                      ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort, double estimatorDT, YoVariableRegistry registry)
   {
      OrientationSensorConfiguration selectedOrientationSensorConfiguration = null;
      if ((orientationSensorConfigurations.size() != 1) || (angularVelocitySensorConfigurations.size() != 1))
         throw new RuntimeException("We are assuming there is only 1 IMU for right now.. Got " + orientationSensorConfigurations.size());

      imuSimulatedDriftYawAcceleration = new YoDouble("imuSimulatedDriftYawAcceleration", registry);
      imuSimulatedDriftYawVelocity = new YoDouble("imuSimulatedDriftYawVelocity", registry);
      imuSimulatedDriftYawAngle = new YoDouble("imuSimulatedDriftYawAngle", registry);
      this.estimatorDT = estimatorDT;
      
      for (OrientationSensorConfiguration orientationSensorConfiguration : orientationSensorConfigurations)
      {
         selectedOrientationSensorConfiguration = orientationSensorConfiguration;
      }

      AngularVelocitySensorConfiguration selectedAngularVelocitySensorConfiguration = null;
      if ((orientationSensorConfigurations.size() != 1) || (angularVelocitySensorConfigurations.size() != 1))
         throw new RuntimeException("We are assuming there is only 1 IMU for right now..");

      for (AngularVelocitySensorConfiguration angularVelocitySensorConfiguration : angularVelocitySensorConfigurations)
      {
         selectedAngularVelocitySensorConfiguration = angularVelocitySensorConfiguration;
      }
      
      this.registry = registry;

      ControlFlowOutputPort<RotationMatrix> orientationSensorOutputPort = selectedOrientationSensorConfiguration.getOutputPort();
      ControlFlowOutputPort<Vector3D> angularVelocitySensorOutputPort = selectedAngularVelocitySensorConfiguration.getOutputPort();

      this.orientationInputPort = createInputPort("orientationInputPort");
      this.angularVelocityInputPort = createInputPort("angularVelocityInputPort");
      this.inverseDynamicsStructureInputPort = createInputPort("inverseDynamicsStructureInputPort");

      controlFlowGraph.connectElements(orientationSensorOutputPort, orientationInputPort);
      controlFlowGraph.connectElements(angularVelocitySensorOutputPort, angularVelocityInputPort);
      controlFlowGraph.connectElements(inverseDynamicsStructureOutputPort, inverseDynamicsStructureInputPort);

      this.estimationLink = inverseDynamicsStructureOutputPort.getData().getEstimationLink();
      this.estimationFrame = inverseDynamicsStructureOutputPort.getData().getEstimationFrame();
      
      this.orientationMeasurementFrame = selectedOrientationSensorConfiguration.getMeasurementFrame();
      this.angularVelocityMeasurementLink = selectedAngularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
      this.angularVelocityMeasurementFrame = selectedAngularVelocitySensorConfiguration.getMeasurementFrame();
      
      this.inverseDynamicsStructureOutputPort = createOutputPort("inverseDynamicsStructureOutputPort");
      this.inverseDynamicsStructureInputPort.setData(inverseDynamicsStructureOutputPort.getData());
      this.inverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
      
      this.orientationOutputPort = new YoFrameQuaternionControlFlowOutputPort(this, "orientationOutput", ReferenceFrame.getWorldFrame(), registry);
      registerOutputPort(orientationOutputPort);
      this.angularVelocityOutputPort = new YoFrameVectorControlFlowOutputPort(this, "angularVelocityOutput", estimationFrame, registry);
      registerOutputPort(angularVelocityOutputPort);

   }

   public void startComputation()
   {
      imuSimulatedDriftYawVelocity.add(imuSimulatedDriftYawAcceleration.getDoubleValue() * estimatorDT);
      imuSimulatedDriftYawAngle.add(imuSimulatedDriftYawVelocity.getDoubleValue() * estimatorDT);
      imuSimulatedDriftYawAngle.set(AngleTools.trimAngleMinusPiToPi(imuSimulatedDriftYawAngle.getDoubleValue()));
      
      convertOrientationAndSetOnOutputPort();      
      convertAngularVelocityAndSetOnOutputPort();   
   }

   private final FrameQuaternion tempOrientationEstimationFrame = new FrameQuaternion(ReferenceFrame.getWorldFrame());

   private final FrameVector3D tempAngularVelocityMeasurementLink = new FrameVector3D();
   private final FrameVector3D tempAngularVelocityEstimationLink = new FrameVector3D();
   private final Twist tempRelativeTwistOrientationMeasFrameToEstFrame = new Twist();

   private final RigidBodyTransform transformFromEstimationFrameToIMUFrame = new RigidBodyTransform();
   private final FrameVector3D relativeAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final RigidBodyTransform transformFromIMUToWorld = new RigidBodyTransform();
   private final RigidBodyTransform transformFromEstimationToWorld = new RigidBodyTransform();
   private final RotationMatrix rotationFromEstimationToWorld = new RotationMatrix();
   
   private final double[] estimationFrameYawPitchRoll = new double[3];
   
   private void convertOrientationAndSetOnOutputPort()
   {
      transformFromIMUToWorld.setRotationAndZeroTranslation(orientationInputPort.getData());

      estimationFrame.getTransformToDesiredFrame(transformFromEstimationFrameToIMUFrame, orientationMeasurementFrame);
      
      transformFromEstimationToWorld.set(transformFromIMUToWorld);
      transformFromEstimationToWorld.multiply(transformFromEstimationFrameToIMUFrame);
      transformFromEstimationToWorld.getRotation(rotationFromEstimationToWorld);
      tempOrientationEstimationFrame.setIncludingFrame(ReferenceFrame.getWorldFrame(), rotationFromEstimationToWorld);
      
      // Introduce simulated IMU drift
      tempOrientationEstimationFrame.getYawPitchRoll(estimationFrameYawPitchRoll);
      estimationFrameYawPitchRoll[0] += imuSimulatedDriftYawAngle.getDoubleValue();
      tempOrientationEstimationFrame.setYawPitchRoll(estimationFrameYawPitchRoll);
      
      orientationOutputPort.setData(tempOrientationEstimationFrame);
   }
   
   private void convertAngularVelocityAndSetOnOutputPort()
   {
      Vector3D measuredAngularVelocityVector3d = angularVelocityInputPort.getData();

      estimationLink.getBodyFixedFrame().getTwistRelativeToOther(angularVelocityMeasurementLink.getBodyFixedFrame(), tempRelativeTwistOrientationMeasFrameToEstFrame);
      tempRelativeTwistOrientationMeasFrameToEstFrame.getAngularPart(relativeAngularVelocity);
      relativeAngularVelocity.changeFrame(estimationFrame);

      tempAngularVelocityMeasurementLink.setIncludingFrame(angularVelocityMeasurementFrame, measuredAngularVelocityVector3d); 
      tempAngularVelocityMeasurementLink.changeFrame(estimationFrame);
      relativeAngularVelocity.add(tempAngularVelocityMeasurementLink);

      tempAngularVelocityEstimationLink.setIncludingFrame(relativeAngularVelocity); // just a copy for clarity
      
      // Introduce simulated IMU drift
      tempAngularVelocityEstimationLink.setZ(tempAngularVelocityEstimationLink.getZ() + imuSimulatedDriftYawVelocity.getDoubleValue());

      angularVelocityOutputPort.setData(tempAngularVelocityEstimationLink);
   }


   public void waitUntilComputationIsDone()
   {
   }

   public ControlFlowOutputPort<FrameQuaternion> getOrientationOutputPort()
   {
      return orientationOutputPort;
   }

   public ControlFlowOutputPort<FrameVector3D> getAngularVelocityOutputPort()
   {
      return angularVelocityOutputPort;
   }

   public ControlFlowOutputPort<FullInverseDynamicsStructure> getInverseDynamicsStructureOutputPort()
   {
      return inverseDynamicsStructureOutputPort;
   }
   
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public void initialize()
   {
      startComputation();
      waitUntilComputationIsDone();
   }

   public void initializeOrientionToActual(FrameQuaternion actualOrientation)
   {
      orientationOutputPort.setData(actualOrientation);
   }
}
