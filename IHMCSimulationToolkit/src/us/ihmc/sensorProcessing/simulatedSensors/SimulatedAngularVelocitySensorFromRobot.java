package us.ihmc.sensorProcessing.simulatedSensors;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.IMUMount;

public class SimulatedAngularVelocitySensorFromRobot extends SimulatedSensor<Tuple3DBasics>
{
   private final IMUMount imuMount;
   
   private final Vector3D angularVelocity = new Vector3D();
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;
   
   private final ControlFlowOutputPort<Vector3D> angularVelocityOutputPort = createOutputPort("angularVelocityOutputPort");

   public SimulatedAngularVelocitySensorFromRobot(String name, IMUMount imuMount, YoVariableRegistry registry)
   {
      this.imuMount = imuMount;
      
      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", null, registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", null, registry);
   }

   public void startComputation()
   {
      imuMount.getAngularVelocityInBody(angularVelocity);
      yoFrameVectorPerfect.set(angularVelocity);
      
      corrupt(angularVelocity);
      yoFrameVectorNoisy.set(angularVelocity);

      angularVelocityOutputPort.setData(angularVelocity);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3D> getAngularVelocityOutputPort()
   {
      return angularVelocityOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
