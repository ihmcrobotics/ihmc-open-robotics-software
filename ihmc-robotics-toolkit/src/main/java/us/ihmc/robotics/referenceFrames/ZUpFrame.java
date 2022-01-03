package us.ihmc.robotics.referenceFrames;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ZUpFrame extends ReferenceFrame
{
   private final ReferenceFrame rootFrame;
   private final ReferenceFrame nonZUpFrame;

   @Deprecated
   public ZUpFrame(ReferenceFrame worldFrame, ReferenceFrame nonZUpFrame, String name)
   {
      this(nonZUpFrame, name);
   }

   public ZUpFrame(ReferenceFrame nonZUpFrame, String name)
   {
      super(name, nonZUpFrame.getRootFrame(), false, true);

      this.rootFrame = nonZUpFrame.getRootFrame();
      this.nonZUpFrame = nonZUpFrame;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      nonZUpFrame.getTransformToDesiredFrame(transformToParent, rootFrame);

      // Compute the yaw rotation matrix while avoiding the computation of the actual yaw-pitch-roll angles.
      double sinPitch = -transformToParent.getM20();
      if (MathTools.epsilonEquals(1.0, Math.abs(sinPitch), 1.0e-12))
      { // pitch = Pi/2 best thing to do is to set the rotation to identity.
         transformToParent.getRotation().setIdentity();
      }
      else
      {
         double cosPitch = Math.sqrt(1.0 - sinPitch * sinPitch);
         double cosYaw = transformToParent.getM00() / cosPitch;
         double sinYaw = transformToParent.getM10() / cosPitch;
         double invNorm = 1.0 / EuclidCoreTools.norm(cosYaw, sinYaw);
         cosYaw *= invNorm;
         sinYaw *= invNorm;
         
         transformToParent.getRotation().setUnsafe(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);
      }
   }
}
