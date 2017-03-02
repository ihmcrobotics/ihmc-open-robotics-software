package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;

public class ZUpFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8615028266128433328L;
   private final Vector3D euler = new Vector3D();
   private final Vector3D translation = new Vector3D();
   private final ReferenceFrame worldFrame;
   private final FramePoint origin;

   //TODO: Combine with ZUpFramePreserveY. Pass in an Axis as to which of X or Y to preserve. By default make it X.
   public ZUpFrame(ReferenceFrame worldFrame, ReferenceFrame nonZUpFrame, String name)
   {
      this(worldFrame, new FramePoint(nonZUpFrame), name);
   }

   public ZUpFrame(ReferenceFrame worldFrame, FramePoint origin, String name)
   {
      super(name, worldFrame, false, false, true);
      this.worldFrame = worldFrame;
      this.origin = new FramePoint(origin);
      
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

      originPoint3d.set(origin.getPoint());
      nonZUpToWorld.transform(originPoint3d);
      translation.set(originPoint3d);
      transformToParent.setTranslation(translation);
   }
}
