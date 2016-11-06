package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimulatedOrientationSensorFromRobot extends SimulatedSensor<Matrix3d>
{
   private final IMUMount imuMount;

   private final Quat4d tempQuaternionOne = new Quat4d();
   private final Quat4d tempQuaternionTwo = new Quat4d();

   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();

   private final Matrix3d rotationMatrix = new Matrix3d();

   private final YoFrameQuaternion yoFrameQuaternionPerfect, yoFrameQuaternionNoisy;
   private final DoubleYoVariable rotationAngleNoise;

   private final ControlFlowOutputPort<Matrix3d> orientationOutputPort = createOutputPort("orientationOutputPort");

   public SimulatedOrientationSensorFromRobot(String name, IMUMount imuMount, YoVariableRegistry registry)
   {
      this.imuMount = imuMount;
      this.yoFrameQuaternionPerfect = new YoFrameQuaternion(name + "Perfect", ReferenceFrame.getWorldFrame(), registry);
      this.yoFrameQuaternionNoisy = new YoFrameQuaternion(name + "Noisy", ReferenceFrame.getWorldFrame(), registry);

      rotationAngleNoise = new DoubleYoVariable(name + "Noise", registry);
   }

   public void startComputation()
   {
      imuMount.getOrientation(rotationMatrix);
      yoFrameQuaternionPerfect.set(rotationMatrix);

      corrupt(rotationMatrix);
      orientationOutputPort.setData(rotationMatrix);
      yoFrameQuaternionNoisy.set(rotationMatrix);

      yoFrameQuaternionPerfect.get(tempQuaternionOne);
      yoFrameQuaternionNoisy.get(tempQuaternionTwo);

      tempQuaternionTwo.inverse();
      tempQuaternionOne.mul(tempQuaternionTwo);

      tempAxisAngle.set(tempQuaternionOne);

      double noiseAngle = tempAxisAngle.getAngle();
      if (noiseAngle > Math.PI)
         noiseAngle = noiseAngle - 2.0 * Math.PI;
      if (noiseAngle < -Math.PI)
         noiseAngle = noiseAngle + 2.0 * Math.PI;

      rotationAngleNoise.set(Math.abs(noiseAngle));
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Matrix3d> getOrientationOutputPort()
   {
      return orientationOutputPort;
   }

   public void initialize()
   {
//    empty
   }
}
