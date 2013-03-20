package us.ihmc.sensorProcessing.simulatedSensors;

import javax.vecmath.Matrix3d;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;

public class SimulatedOrientationSensor extends SimulatedSensor<Matrix3d>
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame frameUsedForPerfectOrientation;

   private final Matrix3d rotationMatrix = new Matrix3d();
   private final YoFrameQuaternion yoFrameQuaternionPerfect, yoFrameQuaternionNoisy;

   private final ControlFlowOutputPort<Matrix3d> orientationOutputPort = createOutputPort();

   public SimulatedOrientationSensor(String name, ReferenceFrame frameUsedForPerfectOrientation, YoVariableRegistry registry)
   {
      this.frameUsedForPerfectOrientation = frameUsedForPerfectOrientation;
      this.yoFrameQuaternionPerfect = new YoFrameQuaternion(name + "Perfect", ReferenceFrame.getWorldFrame(), registry);
      this.yoFrameQuaternionNoisy = new YoFrameQuaternion(name + "Noisy", ReferenceFrame.getWorldFrame(), registry);
   }

   public void startComputation()
   {
      frameUsedForPerfectOrientation.getTransformToDesiredFrame(worldFrame).get(rotationMatrix);
      yoFrameQuaternionPerfect.set(rotationMatrix);

      corrupt(rotationMatrix);
      orientationOutputPort.setData(rotationMatrix);
      yoFrameQuaternionNoisy.set(rotationMatrix);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   public ControlFlowOutputPort<Matrix3d> getOrientationOutputPort()
   {
      return orientationOutputPort;
   }
}
