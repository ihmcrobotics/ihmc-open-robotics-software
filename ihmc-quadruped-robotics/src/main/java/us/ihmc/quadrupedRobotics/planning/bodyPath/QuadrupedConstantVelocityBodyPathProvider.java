package us.ihmc.quadrupedRobotics.planning.bodyPath;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class QuadrupedConstantVelocityBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private final Vector3D planarVelocity = new Vector3D();
   private final FramePose2D initialPose = new FramePose2D();

   public void setInitialPose(FramePose2D initialPose)
   {
      this.initialPose.setIncludingFrame(initialPose);
   }

   public void setPlanarVelocity(Vector3DReadOnly planarVelocity)
   {
      this.planarVelocity.set(planarVelocity);
   }

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
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
