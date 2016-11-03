package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.IMUMount;

public class SimulatedLinearAccelerationSensorFromRobot extends SimulatedSensor<Tuple3d>
{
   private final IMUMount imuMount;

   private final Vector3d linearAcceleration = new Vector3d();
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;

   private final ControlFlowOutputPort<Vector3d> linearAccelerationOutputPort = createOutputPort();

   public SimulatedLinearAccelerationSensorFromRobot(String name, IMUMount imuMount, YoVariableRegistry registry)
   {
      this.imuMount = imuMount;

      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", null, registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", null, registry);
   }

   public void startComputation()
   {
      imuMount.getLinearAccelerationInBody(linearAcceleration);
      yoFrameVectorPerfect.set(linearAcceleration);

      corrupt(linearAcceleration);
      yoFrameVectorNoisy.set(linearAcceleration);

      linearAccelerationOutputPort.setData(linearAcceleration);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3d> getLinearAccelerationOutputPort()
   {
      return linearAccelerationOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
