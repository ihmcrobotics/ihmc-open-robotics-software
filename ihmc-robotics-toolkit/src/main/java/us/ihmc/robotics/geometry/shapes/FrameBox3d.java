package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class FrameBox3d extends FrameShape3d<FrameBox3d, Box3D>
{
   private Box3D box3d;

   public FrameBox3d(FrameBox3d other)
   {
      this(other.referenceFrame, other.box3d);
   }

   public FrameBox3d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Box3D());
      box3d = getGeometryObject();
   }

   public FrameBox3d(ReferenceFrame referenceFrame, Box3D box3d)
   {
      super(referenceFrame, new Box3D(box3d));
      this.box3d = getGeometryObject();
   }

   public FrameBox3d(ReferenceFrame referenceFrame, double lengthX, double widthY, double heightZ)
   {
      super(referenceFrame, new Box3D(lengthX, widthY, heightZ));
      box3d = getGeometryObject();
   }

   public FrameBox3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double lengthX, double widthY, double heightZ)
   {
      super(referenceFrame, new Box3D(configuration, lengthX, widthY, heightZ));
      box3d = getGeometryObject();
   }

   public FrameBox3d(ReferenceFrame referenceFrame, Pose3D pose, double lengthX, double widthY, double heightZ)
   {
      super(referenceFrame, new Box3D(pose.getPosition(), pose.getOrientation(), lengthX, widthY, heightZ));
      box3d = getGeometryObject();
   }

   public Box3D getBox3d()
   {
      return box3d;
   }

   public FramePoint3D getCenter()
   {
      FramePoint3D ret = new FramePoint3D(referenceFrame);
      getCenter(ret);
      return ret;
   }

   public void getCenter(FramePoint3D pointToPack)
   {
      pointToPack.setIncludingFrame(referenceFrame, box3d.getPosition());
   }

   public FramePoint3D getCenterCopy()
   {
      FramePoint3D ret = new FramePoint3D(referenceFrame);
      getCenter(ret);

      return ret;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(this.box3d.getPose());
   }

   public RigidBodyTransform getTransformCopy()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransform(ret);

      return ret;
   }

   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(this.box3d.getOrientation());
   }

   public RotationMatrix getRotationCopy()
   {
      RotationMatrix ret = new RotationMatrix();
      getRotation(ret);

      return ret;
   }

   public void setPose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      box3d.getPose().set(orientation, position);
   }
   
   public void setPose(Pose3DReadOnly pose)
   {
      box3d.getPose().set(pose.getOrientation(), pose.getPosition());
   }
   
   public void setTransform(RigidBodyTransform transform3D)
   {
      box3d.getPose().set(transform3D);
   }

   public void getFramePose(FramePose3D framePoseToPack)
   {
      framePoseToPack.setToZero(referenceFrame);
      box3d.getPose().set(framePoseToPack);
   }

   public void scale(double scale)
   {
      box3d.scale(scale);
   }

   public void computeVertices(Point3D[] vertices)
   {
      box3d.getVertices(vertices);
   }
}
