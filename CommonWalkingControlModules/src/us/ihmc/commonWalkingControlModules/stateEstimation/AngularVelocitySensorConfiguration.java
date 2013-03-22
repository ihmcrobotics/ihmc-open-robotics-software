package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
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

   public DenseMatrix64F getBiasCovariance(PortType port)
   {
      return biasCovariances.get(port);
   }

   public ArrayList<NewAngularVelocitySensorConfiguration> getNewAngularVelocitySensorConfiguration()
   {
      ArrayList<NewAngularVelocitySensorConfiguration> ret = new ArrayList<NewAngularVelocitySensorConfiguration>();

      for (PortType measurementPort : measurementPorts)
      {
         ControlFlowOutputPort<Vector3d> outputPort = (ControlFlowOutputPort<Vector3d>) measurementPort;
         String name = sensorNames.get(measurementPort);
         ReferenceFrame measurementFrame = measurementFrames.get(measurementPort);
         RigidBody measurementLink = rigidBodies.get(measurementPort);
         DenseMatrix64F angularVelocityNoiseCovariance = covariances.get(measurementPort);
         DenseMatrix64F biasProcessNoiseCovariance = biasCovariances.get(measurementPort);
         NewAngularVelocitySensorConfiguration newAngularVelocitySensorConfiguration = new NewAngularVelocitySensorConfiguration(outputPort, name,
                                                                                          measurementLink, measurementFrame, angularVelocityNoiseCovariance,
                                                                                          biasProcessNoiseCovariance);

         ret.add(newAngularVelocitySensorConfiguration);
      }

      return ret;
   }
}
