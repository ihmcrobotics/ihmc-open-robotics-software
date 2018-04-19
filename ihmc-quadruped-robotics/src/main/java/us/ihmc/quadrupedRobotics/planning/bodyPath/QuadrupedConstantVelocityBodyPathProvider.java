package us.ihmc.quadrupedRobotics.planning.bodyPath;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class QuadrupedConstantVelocityBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble bodyYaw = new YoDouble("bodyYaw", registry);
   private final YoFrameYawPitchRoll bodyOrientation = new YoFrameYawPitchRoll("bodyOrientation", worldFrame, registry);;
   private final FramePoint3D supportCentroid = new FramePoint3D();

   private final Vector3D planarVelocity = new Vector3D();
   private final FramePose2D initialPose = new FramePose2D();
   private final ReferenceFrame bodyZUpFrame;
   private final ReferenceFrame supportFrame;
   private final YoDouble timestamp, previousTimestamp;
   private final double controlDT;

   public QuadrupedConstantVelocityBodyPathProvider(QuadrupedReferenceFrames referenceFrames, double controlDT, YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.controlDT = controlDT;
      this.timestamp = timestamp;
      this.previousTimestamp = new YoDouble("previousTimestamp", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      previousTimestamp.set(timestamp.getDoubleValue());

      bodyOrientation.setFromReferenceFrame(bodyZUpFrame);
      bodyYaw.set(bodyOrientation.getYaw().getDoubleValue());
      initialPose.setYaw(bodyYaw.getDoubleValue());

      setInitialPose();
   }

   private void setInitialPose()
   {
      supportCentroid.setToZero(supportFrame);
      supportCentroid.changeFrame(worldFrame);

      initialPose.setPosition(supportCentroid);
      initialPose.setYaw(bodyYaw.getDoubleValue());
   }

   public void setPlanarVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityYaw)
   {
      this.planarVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityYaw);
   }

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
   {
      // update body orientation
      if(Double.isNaN(controlDT))
      {
         double effectiveDT = timestamp.getDoubleValue() - previousTimestamp.getDoubleValue();
         previousTimestamp.set(timestamp.getDoubleValue());
         updateBodyOrientation(effectiveDT);
      }
      else
      {
         updateBodyOrientation(controlDT);
      }

      setInitialPose();
      extrapolatePose(time, poseToPack, initialPose, planarVelocity);
   }

   private void updateBodyOrientation(double dt)
   {
      bodyYaw.add(planarVelocity.getZ() * dt);
      bodyOrientation.setYawPitchRoll(bodyYaw.getDoubleValue(), 0.0, 0.0);
   }

   private static void extrapolatePose(double time, FramePose2D poseToPack, FramePose2D initialPose, Vector3D planarVelocity)
   {
      double a0 = initialPose.getYaw();
      double x0 = initialPose.getX();
      double y0 = initialPose.getY();

      // initialize forward, lateral, and rotational velocity in pose frame
      double u = planarVelocity.getX();
      double v = planarVelocity.getY();
      double phi = planarVelocity.getZ();

      // compute extrapolated pose assuming a constant planar velocity
      double a, x, y;
      double epsilon = 0.001;
      if (Math.abs(phi) > epsilon)
      {
         a = a0 + phi * time;
         x = x0 + u / phi * (Math.sin(a) - Math.sin(a0)) + v / phi * (Math.cos(a) - Math.cos(a0));
         y = y0 - u / phi * (Math.cos(a) - Math.cos(a0)) + v / phi * (Math.sin(a) - Math.sin(a0));
      }
      else
      {
         a = a0;
         x = x0 + (u * Math.cos(a) - v * Math.sin(a)) * time;
         y = y0 + (u * Math.sin(a) + v * Math.cos(a)) * time;
      }

      poseToPack.setX(x);
      poseToPack.setY(y);
      poseToPack.setYaw(a);
   }
}
