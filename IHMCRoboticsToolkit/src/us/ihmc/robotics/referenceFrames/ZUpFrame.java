package us.ihmc.robotics.referenceFrames;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class ZUpFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8615028266128433328L;
   private final Vector3d euler = new Vector3d();
   private final Vector3d translation = new Vector3d();
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

   private final Matrix3d nonZUpToWorldRotation = new Matrix3d();
   private final Point3d originPoint3d = new Point3d();
   private final RigidBodyTransform nonZUpToWorld = new RigidBodyTransform();

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      //TODO: Combine with RotationTools.removePitchAndRollFromTransform(). 
      origin.getReferenceFrame().getTransformToDesiredFrame(nonZUpToWorld, worldFrame);
      nonZUpToWorld.getRotation(nonZUpToWorldRotation);

      double yaw = RotationTools.computeYaw(nonZUpToWorldRotation);
      euler.set(0.0, 0.0, yaw);
      transformToParent.setRotationEulerAndZeroTranslation(euler);

      originPoint3d.set(origin.getPoint());
      nonZUpToWorld.transform(originPoint3d);
      translation.set(originPoint3d);
      transformToParent.setTranslation(translation);
   }
}
