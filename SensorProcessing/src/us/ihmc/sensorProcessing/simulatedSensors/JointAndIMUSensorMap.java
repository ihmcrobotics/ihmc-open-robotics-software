package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.IMUDefinition;

public class JointAndIMUSensorMap
{
   private final LinkedHashMap<OneDoFJoint, ControlFlowOutputPort<double[]>> jointPositionSensors = new LinkedHashMap<OneDoFJoint,
                                                                                                     ControlFlowOutputPort<double[]>>();
   private final LinkedHashMap<OneDoFJoint, ControlFlowOutputPort<double[]>> jointVelocitySensors = new LinkedHashMap<OneDoFJoint,
                                                                                                     ControlFlowOutputPort<double[]>>();

   private final LinkedHashMap<IMUDefinition, ControlFlowOutputPort<RotationMatrix>> orientationSensors = new LinkedHashMap<IMUDefinition,
                                                                                                       ControlFlowOutputPort<RotationMatrix>>();
   private final LinkedHashMap<IMUDefinition, ControlFlowOutputPort<Vector3D>> angularVelocitySensors = new LinkedHashMap<IMUDefinition,
                                                                                                           ControlFlowOutputPort<Vector3D>>();
   private final LinkedHashMap<IMUDefinition, ControlFlowOutputPort<Vector3D>> linearAccelerationSensors = new LinkedHashMap<IMUDefinition,
                                                                                                              ControlFlowOutputPort<Vector3D>>();

   public JointAndIMUSensorMap()
   {
   }

   public Map<OneDoFJoint, ControlFlowOutputPort<double[]>> getJointPositionSensors()
   {
      return jointPositionSensors;
   }

   public Map<OneDoFJoint, ControlFlowOutputPort<double[]>> getJointVelocitySensors()
   {
      return jointVelocitySensors;
   }

   public Map<IMUDefinition, ControlFlowOutputPort<RotationMatrix>> getOrientationSensors()
   {
      return orientationSensors;
   }

   public Map<IMUDefinition, ControlFlowOutputPort<Vector3D>> getAngularVelocitySensors()
   {
      return angularVelocitySensors;
   }

   public Map<IMUDefinition, ControlFlowOutputPort<Vector3D>> getLinearAccelerationSensors()
   {
      return linearAccelerationSensors;
   }

   public ControlFlowOutputPort<double[]> getJointPositionSensorPort(OneDoFJoint oneDoFJoint)
   {
      return jointPositionSensors.get(oneDoFJoint);
   }

   public ControlFlowOutputPort<double[]> getJointVelocitySensorPort(OneDoFJoint oneDoFJoint)
   {
      return jointVelocitySensors.get(oneDoFJoint);
   }

   public ControlFlowOutputPort<RotationMatrix> getOrientationSensorPort(IMUDefinition imuDefinition)
   {
      return orientationSensors.get(imuDefinition);
   }

   public ControlFlowOutputPort<Vector3D> getAngularVelocitySensorPort(IMUDefinition imuDefinition)
   {
      return angularVelocitySensors.get(imuDefinition);
   }

   public ControlFlowOutputPort<Vector3D> getLinearAccelerationSensorPort(IMUDefinition imuDefinition)
   {
      return linearAccelerationSensors.get(imuDefinition);
   }

   public void addJointPositionSensorPort(OneDoFJoint oneDoFJoint, ControlFlowOutputPort<double[]> jointPositionSensorPort)
   {
      jointPositionSensors.put(oneDoFJoint, jointPositionSensorPort);
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, ControlFlowOutputPort<double[]> jointVelocitySensorPort)
   {
      jointVelocitySensors.put(oneDoFJoint, jointVelocitySensorPort);
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, ControlFlowOutputPort<RotationMatrix> orientationSensorPort)
   {
      orientationSensors.put(imuDefinition, orientationSensorPort);
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, ControlFlowOutputPort<Vector3D> angularVelocitySensorPort)
   {
      angularVelocitySensors.put(imuDefinition, angularVelocitySensorPort);
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, ControlFlowOutputPort<Vector3D> linearAccelerationSensorPort)
   {
      linearAccelerationSensors.put(imuDefinition, linearAccelerationSensorPort);
   }

   public Collection<ControlFlowOutputPort<RotationMatrix>> getOrientationOutputPorts()
   {
      return orientationSensors.values();
   }

   public Collection<ControlFlowOutputPort<Vector3D>> getAngularVelocityOutputPorts()
   {
      return angularVelocitySensors.values();
   }

   public Collection<ControlFlowOutputPort<Vector3D>> getLinearAccelerationOutputPorts()
   {
      return linearAccelerationSensors.values();
   }

}
