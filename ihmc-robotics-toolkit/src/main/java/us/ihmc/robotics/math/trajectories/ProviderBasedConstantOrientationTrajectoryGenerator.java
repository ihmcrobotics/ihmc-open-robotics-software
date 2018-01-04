package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class ProviderBasedConstantOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame referenceFrame;
   private final OrientationProvider orientationProvider;
   private final YoDouble time;
   private final YoDouble finalTime;

   public ProviderBasedConstantOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, OrientationProvider orientationProvider,
         double finalTime, YoVariableRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.referenceFrame = referenceFrame;
      this.orientationProvider = orientationProvider;
      this.finalTime = new YoDouble("finalTime", registry);
      this.time = new YoDouble("time", registry);
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

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationProvider.getOrientation(orientationToPack);
      orientationToPack.changeFrame(referenceFrame);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(referenceFrame);
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(referenceFrame);
   }

   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}