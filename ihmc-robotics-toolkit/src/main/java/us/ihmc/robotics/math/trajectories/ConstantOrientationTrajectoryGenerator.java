package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ConstantOrientationTrajectoryGenerator implements OrientationTrajectoryGenerator
{
   private final YoRegistry registry;
   private final YoDouble finalTime;
   private final YoDouble time;
   private final OrientationProvider orientationProvider;
   private final YoFrameQuaternion orientation;

   public ConstantOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, OrientationProvider orientationProvider, double finalTime,
                                                 YoRegistry parentRegistry)
   {
      MathTools.checkIntervalContains(finalTime, 0.0, Double.POSITIVE_INFINITY);

      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.orientationProvider = orientationProvider;
      this.orientation = new YoFrameQuaternion("orientation", referenceFrame, registry);
      this.finalTime = new YoDouble("finalTime", registry);
      this.time = new YoDouble("time", registry);
      this.finalTime.set(finalTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      time.set(0.0);
      FrameQuaternion orientationToPack = new FrameQuaternion(ReferenceFrame.getWorldFrame());
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

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(orientation.getReferenceFrame());
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(orientation.getReferenceFrame());
   }

   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }
}
