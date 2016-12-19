package us.ihmc.robotics.referenceFrames;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class ZUpPreserveYReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -1454797908129819243L;
   private final Vector3d translation = new Vector3d();
   private final ReferenceFrame worldFrame;
   private final FramePoint origin;

   public ZUpPreserveYReferenceFrame(ReferenceFrame worldFrame, ReferenceFrame nonZUpFrame, String name)
   {
      this(worldFrame, new FramePoint(nonZUpFrame), name);
   }

   public ZUpPreserveYReferenceFrame(ReferenceFrame worldFrame, FramePoint origin, String name)
   {
      super(name, worldFrame, false, false, true);
      this.worldFrame = worldFrame;
      this.origin = new FramePoint(origin);
      
      this.update();
   }

   private final Matrix3d nonZUpToWorldRotation = new Matrix3d();
   private final Matrix3d zUpToWorldRotation = new Matrix3d();
   private final Point3d originPoint3d = new Point3d();
   private final RigidBodyTransform nonZUpToWorld = new RigidBodyTransform();
   
   private final Vector3d xAxis = new Vector3d();
   private final Vector3d yAxis = new Vector3d();
   private final Vector3d zAxis = new Vector3d();
   
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      //TODO: Combine with RotationTools.removePitchAndRollFromTransform(). 
      origin.getReferenceFrame().getTransformToDesiredFrame(nonZUpToWorld, worldFrame);
      nonZUpToWorld.getRotation(nonZUpToWorldRotation);

      double yAxisX = nonZUpToWorldRotation.getM01();
      double yAxisY = nonZUpToWorldRotation.getM11();
      yAxis.set(yAxisX, yAxisY, 0.0);
      yAxis.normalize();
      
      zAxis.set(0.0, 0.0, 1.0);
      xAxis.cross(yAxis, zAxis);
      
      zUpToWorldRotation.setColumn(0, xAxis);
      zUpToWorldRotation.setColumn(1, yAxis);
      zUpToWorldRotation.setColumn(2, zAxis);
     
      transformToParent.setRotation(zUpToWorldRotation);

      originPoint3d.set(origin.getPoint());
      nonZUpToWorld.transform(originPoint3d);
      translation.set(originPoint3d);
      transformToParent.setTranslation(translation);
   }
}
