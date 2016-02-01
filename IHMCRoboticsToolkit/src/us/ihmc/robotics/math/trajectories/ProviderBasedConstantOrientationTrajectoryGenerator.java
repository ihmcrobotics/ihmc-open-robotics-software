package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;


public class ProviderBasedConstantOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame referenceFrame;
   private final OrientationProvider orientationProvider;
   private final DoubleYoVariable time;
   private final DoubleYoVariable finalTime;

   public ProviderBasedConstantOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, OrientationProvider orientationProvider,
         double finalTime, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIfInRange(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.referenceFrame = referenceFrame;
      this.orientationProvider = orientationProvider;
      this.finalTime = new DoubleYoVariable("finalTime", registry);
      this.time = new DoubleYoVariable("time", registry);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
   }

   public void compute(double time)
   {
      this.time.set(time);
   }

   public boolean isDone()
   {
      return time.getDoubleValue() > finalTime.getDoubleValue();
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationProvider.get(orientationToPack);
      orientationToPack.changeFrame(referenceFrame);
   }

   public void packAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(referenceFrame);
   }

   public void packAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(referenceFrame);
   }

   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      get(orientationToPack);
      packAngularVelocity(angularVelocityToPack);
      packAngularAcceleration(angularAccelerationToPack);
   }
}