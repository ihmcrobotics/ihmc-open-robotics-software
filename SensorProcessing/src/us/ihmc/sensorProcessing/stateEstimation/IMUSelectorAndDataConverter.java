package us.ihmc.sensorProcessing.stateEstimation;

import java.util.Collection;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class IMUSelectorAndDataConverter extends AbstractControlFlowElement
{
   private final ControlFlowInputPort<Matrix3d> orientationInputPort;
   private final ControlFlowInputPort<Vector3d> angularVelocityInputPort;
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final ControlFlowOutputPort<FrameOrientation> orientationOutputPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityOutputPort;
   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;

   private final ReferenceFrame desiredOutputFrame;

   private final RigidBody estimationLink;
   private final RigidBody angularVelocityMeasurementLink;

   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame orientationMeasurementFrame;
   private final ReferenceFrame angularVelocityMeasurementFrame;

   public IMUSelectorAndDataConverter(ControlFlowGraph controlFlowGraph, Collection<OrientationSensorConfiguration> orientationSensorConfigurations,
                                      Collection<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations,
                                      ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort)    // throws Exception
   {
      OrientationSensorConfiguration selectedOrientationSensorConfiguration = null;
      if ((orientationSensorConfigurations.size() != 1) || (angularVelocitySensorConfigurations.size() != 1))
         throw new RuntimeException("We are assuming there is only 1 IMU for right now..");

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

      ControlFlowOutputPort<Matrix3d> orientationSensorOutputPort = selectedOrientationSensorConfiguration.getOutputPort();
      ControlFlowOutputPort<Vector3d> angularVelocitySensorOutputPort = selectedAngularVelocitySensorConfiguration.getOutputPort();

      this.orientationInputPort = createInputPort("orientationInputPort");
      this.angularVelocityInputPort = createInputPort("angularVelocityInputPort");
      this.inverseDynamicsStructureInputPort = createInputPort("inverseDynamicsStructureInputPort");

      controlFlowGraph.connectElements(orientationSensorOutputPort, orientationInputPort);
      controlFlowGraph.connectElements(angularVelocitySensorOutputPort, angularVelocityInputPort);
      controlFlowGraph.connectElements(inverseDynamicsStructureOutputPort, inverseDynamicsStructureInputPort);

      this.orientationOutputPort = createOutputPort("orientationOutputPort");
      this.angularVelocityOutputPort = createOutputPort("angularVelocityOutputPort");
      this.inverseDynamicsStructureOutputPort = createOutputPort("inverseDynamicsStructureOutputPort");

      this.estimationLink = inverseDynamicsStructureOutputPort.getData().getEstimationLink();
      this.estimationFrame = inverseDynamicsStructureOutputPort.getData().getEstimationFrame();

      this.orientationMeasurementFrame = selectedOrientationSensorConfiguration.getMeasurementFrame();
      this.angularVelocityMeasurementLink = selectedAngularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();
      this.angularVelocityMeasurementFrame = selectedAngularVelocitySensorConfiguration.getMeasurementFrame();

      this.desiredOutputFrame = ReferenceFrame.getWorldFrame();

      this.inverseDynamicsStructureInputPort.setData(inverseDynamicsStructureOutputPort.getData());
      this.inverseDynamicsStructureOutputPort.setData(inverseDynamicsStructureInputPort.getData());
   }

   public void startComputation()
   {
      convertRawDataAndSetOnOutputPort(desiredOutputFrame, orientationInputPort.getData(), angularVelocityInputPort.getData());
   }

   private final FrameOrientation tempOrientationMeasurementFrame = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FrameOrientation tempOrientationEstimationFrame = new FrameOrientation(ReferenceFrame.getWorldFrame());

   private final FrameVector tempAngularVelocityMeasurementLink = new FrameVector();
   private final FrameVector tempAngularVelocityEstimationLink = new FrameVector();
//   private final Twist tempTwistAngularVelocityMeasurementLink = new Twist();
//   private final Twist tempTwistAngularVelocityEstimationLink = new Twist();


   private final Twist tempRelativeTwistOrientationMeasFrameToEstFrame = new Twist();

   private final Transform3D transformOrientationMeasFrameToEstFrame = new Transform3D();
   private final FrameVector relativeAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   

   private void convertRawDataAndSetOnOutputPort(ReferenceFrame desiredOutputFrame, Matrix3d rawOrientationData, Vector3d rawAngularVelocityData)
   {
      // Orientation part
      tempOrientationMeasurementFrame.set(ReferenceFrame.getWorldFrame(), orientationInputPort.getData());

      estimationFrame.getTransformToDesiredFrame(transformOrientationMeasFrameToEstFrame, orientationMeasurementFrame);
      
      tempOrientationEstimationFrame.set(tempOrientationMeasurementFrame.applyTransformCopy(transformOrientationMeasFrameToEstFrame));
      
      // set on port
      orientationOutputPort.setData(tempOrientationEstimationFrame);
      
      // Angular velocity part
      Vector3d measuredAngularVelocityVector3d = angularVelocityInputPort.getData();
      TwistCalculator twistCalculator = inverseDynamicsStructureInputPort.getData().getTwistCalculator();
      
      twistCalculator.packRelativeTwist(tempRelativeTwistOrientationMeasFrameToEstFrame, angularVelocityMeasurementLink, estimationLink);
      tempRelativeTwistOrientationMeasFrameToEstFrame.packAngularPart(relativeAngularVelocity);
      relativeAngularVelocity.changeFrame(estimationFrame);
      
      tempAngularVelocityMeasurementLink.set(ReferenceFrame.getWorldFrame(), measuredAngularVelocityVector3d);  
      tempAngularVelocityMeasurementLink.changeFrame(estimationFrame);
      tempAngularVelocityMeasurementLink.add(relativeAngularVelocity);
      
      tempAngularVelocityEstimationLink.setAndChangeFrame(tempAngularVelocityMeasurementLink);
      tempAngularVelocityEstimationLink.changeFrame(desiredOutputFrame);
      
      // set on port
      angularVelocityOutputPort.setData(tempAngularVelocityEstimationLink);
   }

   public void convertOrientationMeasurementLinkAToLinkB()
   {
	   
   }
   

   public void waitUntilComputationIsDone()
   {
      // empty
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
      return inverseDynamicsStructureOutputPort;
   }

}
