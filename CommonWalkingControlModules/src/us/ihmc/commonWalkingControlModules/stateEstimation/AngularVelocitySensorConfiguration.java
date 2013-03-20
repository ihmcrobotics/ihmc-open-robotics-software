package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.HashMap;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.ControlFlowPort;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class AngularVelocitySensorConfiguration<PortType extends ControlFlowPort<Vector3d>> extends SensorConfiguration<Vector3d, PortType>
{
   private static final int VECTOR3D_LENGTH = 3;

   private final Map<PortType, ReferenceFrame> measurementFrames = new HashMap<PortType, ReferenceFrame>();
   private final Map<PortType, RigidBody> rigidBodies = new HashMap<PortType, RigidBody>();
   private final Map<PortType, DenseMatrix64F> biasCovariances = new HashMap<PortType, DenseMatrix64F>();

   public AngularVelocitySensorConfiguration()
   {
      super(VECTOR3D_LENGTH);
   }

   public void addSensor(PortType measurementPort, ReferenceFrame measurementFrame, RigidBody rigidBody, String sensorName, DenseMatrix64F covariance,
                         DenseMatrix64F biasCovariance)
   {
      measurementPorts.add(measurementPort);
      measurementFrames.put(measurementPort, measurementFrame);
      rigidBodies.put(measurementPort, rigidBody);
      sensorNames.put(measurementPort, sensorName);
      covariances.put(measurementPort, covariance);
      biasCovariances.put(measurementPort, biasCovariance);
   }

   public ReferenceFrame getReferenceFrame(PortType port)
   {
      return measurementFrames.get(port);
   }

   public RigidBody getRigidBody(PortType port)
   {
      return rigidBodies.get(port);
   }

   public DenseMatrix64F getBiasCovariance(ControlFlowOutputPort<Vector3d> port)
   {
      return biasCovariances.get(port);
   }
}
