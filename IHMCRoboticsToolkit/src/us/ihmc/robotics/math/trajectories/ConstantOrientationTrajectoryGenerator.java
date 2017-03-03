package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;


public class ConstantOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable finalTime;
   private final DoubleYoVariable time;
   private final OrientationProvider orientationProvider;
   private final YoFrameQuaternion orientation;

   public ConstantOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, OrientationProvider orientationProvider, double finalTime,
                                                 YoVariableRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.orientationProvider = orientationProvider;
      this.orientation = new YoFrameQuaternion("orientation", referenceFrame, registry);
      this.finalTime = new DoubleYoVariable("finalTime", registry);
      this.time = new DoubleYoVariable("time", registry);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
      FrameOrientation orientationToPack = new FrameOrientation(ReferenceFrame.getWorldFrame());
      orientationProvider.getOrientation(orientationToPack);
      orientationToPack.changeFrame(orientation.getReferenceFrame());
      orientation.set(orientationToPack);
   }

   public void compute(double time)
   {
      this.time.set(time);
   }

   public boolean isDone()
   {
      return time.getDoubleValue() > finalTime.getDoubleValue();
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(orientation.getReferenceFrame());
   }

   public void getAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(orientation.getReferenceFrame());
   }

   public void getAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}
