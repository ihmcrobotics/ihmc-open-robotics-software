package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ZUpFrame extends ReferenceFrame
{
   private final Vector3D euler = new Vector3D();
   private final Vector3D translation = new Vector3D();
   private final ReferenceFrame worldFrame;
   private final FramePoint3D origin;

   //TODO: Combine with ZUpFramePreserveY. Pass in an Axis as to which of X or Y to preserve. By default make it X.
   public ZUpFrame(ReferenceFrame worldFrame, ReferenceFrame nonZUpFrame, String name)
   {
      this(worldFrame, new FramePoint3D(nonZUpFrame), name);
   }

   public ZUpFrame(ReferenceFrame worldFrame, FramePoint3D origin, String name)
   {
      super(name, worldFrame, false, true);
      this.worldFrame = worldFrame;
      this.origin = new FramePoint3D(origin);
      
      this.update();
   }

   private final RotationMatrix nonZUpToWorldRotation = new RotationMatrix();
   private final Point3D originPoint3d = new Point3D();
   private final RigidBodyTransform nonZUpToWorld = new RigidBodyTransform();

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      //TODO: Combine with RotationTools.removePitchAndRollFromTransform(). 
      origin.getReferenceFrame().getTransformToDesiredFrame(nonZUpToWorld, worldFrame);
      nonZUpToWorld.getRotation(nonZUpToWorldRotation);

      double yaw = nonZUpToWorldRotation.getYaw();
      euler.set(0.0, 0.0, yaw);
      transformToParent.setRotationEulerAndZeroTranslation(euler);

      originPoint3d.set(origin);
      nonZUpToWorld.transform(originPoint3d);
      translation.set(originPoint3d);
      transformToParent.setTranslation(translation);
   }
}
