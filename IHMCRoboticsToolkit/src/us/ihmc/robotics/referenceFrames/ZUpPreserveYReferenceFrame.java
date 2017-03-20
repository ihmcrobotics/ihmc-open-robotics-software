package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;

public class ZUpPreserveYReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -1454797908129819243L;
   private final Vector3D translation = new Vector3D();
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

   private final RotationMatrix nonZUpToWorldRotation = new RotationMatrix();
   private final RotationMatrix zUpToWorldRotation = new RotationMatrix();
   private final Point3D originPoint3d = new Point3D();
   private final RigidBodyTransform nonZUpToWorld = new RigidBodyTransform();
   
   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D();
   
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
      
      zUpToWorldRotation.setColumns(xAxis, yAxis, zAxis);
     
      transformToParent.setRotation(zUpToWorldRotation);

      originPoint3d.set(origin.getPoint());
      nonZUpToWorld.transform(originPoint3d);
      translation.set(originPoint3d);
      transformToParent.setTranslation(translation);
   }
}
