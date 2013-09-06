package us.ihmc.sensorProcessing.stateEstimation;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class IMUSelectorAndDataWrapper extends AbstractControlFlowElement
{

   private final ControlFlowInputPort<Matrix3d> orientationInputPort;
   private final ControlFlowInputPort<Vector3d> angularVelocityInputPort;

   private final ControlFlowOutputPort<FrameOrientation> orientationOutputPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityOutputPort;

   private final ReferenceFrame desiredOutputFrame;
   private final FrameOrientation orientationIMUInOutputFrame;
   private final FrameVector angularVelocityIMUInOutputFrame;
   private final FrameOrientation orientationLinkOfIMU;
   private final FrameVector angularVelocityLinkOfIMU;

   public IMUSelectorAndDataWrapper(ControlFlowGraph controlFlowGraph, JointAndIMUSensorMap jointAndIMUSensorMap) // throws Exception
   {
//      if (jointAndIMUSensorMap.getOrientationSensors().size() != 1)
//      {
//         throw new Exception("Please select the IMU you trust and assume to be perfect in " + getClass());
//      }

      IMUDefinition selectedIMU = jointAndIMUSensorMap.getOrientationSensors().entrySet().iterator().next().getKey();
      ControlFlowOutputPort<Matrix3d> orientationSensorOutputPort = jointAndIMUSensorMap.getOrientationSensorPort(selectedIMU);
      ControlFlowOutputPort<Vector3d> angularVelocitySensorOutputPort = jointAndIMUSensorMap.getAngularVelocitySensorPort(selectedIMU);
      this.orientationInputPort = createInputPort("orientationInputPort");
      this.angularVelocityInputPort = createInputPort("angularVelocityInputPort");

      controlFlowGraph.connectElements(orientationSensorOutputPort, orientationInputPort);
      controlFlowGraph.connectElements(angularVelocitySensorOutputPort, angularVelocityInputPort);

      this.orientationOutputPort = createOutputPort("orientationOutputPort");
      this.angularVelocityOutputPort = createOutputPort("angularVelocityOutputPort");

      // initialize ports and data, put data on ports for first time
      this.orientationIMUInOutputFrame = null;
      this.angularVelocityIMUInOutputFrame = null;
      this.desiredOutputFrame = ReferenceFrame.getWorldFrame();
      orientationLinkOfIMU = new FrameOrientation(selectedIMU.getRigidBody().getBodyFixedFrame());
      angularVelocityLinkOfIMU = new FrameVector(selectedIMU.getRigidBody().getBodyFixedFrame());
      convertRawDataAndSetOnOutputPort(desiredOutputFrame, orientationSensorOutputPort.getData(), angularVelocitySensorOutputPort.getData());
   }

   public void startComputation()
   {
      convertRawDataAndSetOnOutputPort(desiredOutputFrame, orientationInputPort.getData(), angularVelocityInputPort.getData());
   }

   private void convertRawDataAndSetOnOutputPort(ReferenceFrame desiredOutputFrame, Matrix3d rawOrientationData, Vector3d rawAngularVelocityData)
   {
      orientationLinkOfIMU.set(rawOrientationData);
      angularVelocityLinkOfIMU.set(rawAngularVelocityData);

      orientationIMUInOutputFrame.set(orientationLinkOfIMU.changeFrameCopy(desiredOutputFrame));
      angularVelocityIMUInOutputFrame.set(angularVelocityLinkOfIMU.changeFrameCopy(desiredOutputFrame));

      orientationOutputPort.setData(orientationIMUInOutputFrame);
      angularVelocityOutputPort.setData(angularVelocityIMUInOutputFrame);
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

}
