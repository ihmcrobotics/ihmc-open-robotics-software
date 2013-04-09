package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Vector3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class SimulatedPointVelocitySensorFromRobot extends SimulatedSensor<Vector3d>
{
   private final KinematicPoint kinematicPoint;
   
   private final Vector3d pointVelocity = new Vector3d();
   private final YoFrameVector yoFrameVectorPerfect, yoFrameVectorNoisy;

   private final ControlFlowOutputPort<Vector3d> pointVelocityOutputPort = createOutputPort();

   public SimulatedPointVelocitySensorFromRobot(String name, KinematicPoint kinematicPoint, YoVariableRegistry registry)
   {
      this.kinematicPoint = kinematicPoint;

      this.yoFrameVectorPerfect = new YoFrameVector(name + "Perfect", ReferenceFrame.getWorldFrame(), registry);
      this.yoFrameVectorNoisy = new YoFrameVector(name + "Noisy", ReferenceFrame.getWorldFrame(), registry);
   }

   public void startComputation()
   {
      kinematicPoint.getVelocity(pointVelocity);
      yoFrameVectorPerfect.set(pointVelocity);

      corrupt(pointVelocity);
      yoFrameVectorNoisy.set(pointVelocity);
      
      pointVelocityOutputPort.setData(pointVelocity);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Vector3d> getPointVelocityOutputPort()
   {
      return pointVelocityOutputPort;
   }
}

