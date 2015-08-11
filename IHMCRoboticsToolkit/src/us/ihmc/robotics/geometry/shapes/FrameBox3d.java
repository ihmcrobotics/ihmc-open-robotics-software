package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.shapes.Box3d.FaceName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;

public class FrameBox3d extends FrameShape3d
{
   private ReferenceFrame referenceFrame;
   private Box3d box3d;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   public Box3d getBox3d()
   {
      return box3d;
   }

   public FramePoint getCenter()
   {
      FramePoint ret = new FramePoint(referenceFrame);
      getCenter(ret);
      return ret;
   }

   public void getCenter(FramePoint pointToPack)
   {
      pointToPack.setToZero(referenceFrame);
      box3d.getCenter(pointToPack.getPoint());
   }

   public FramePoint getCenterCopy()
   {
      FramePoint ret = new FramePoint(referenceFrame);
      getCenter(ret);

      return ret;
   }

   public double getDimension(Direction direction)
   {
      return box3d.getDimension(direction);
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      this.box3d.getTransform(transformToPack);
   }

   public RigidBodyTransform getTransformCopy()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransform(ret);

      return ret;
   }

   public void getRotation(Matrix3d rotationMatrixToPack)
   {
      this.box3d.transform.get(rotationMatrixToPack);
   }

   public Matrix3d getRotationCopy()
   {
      Matrix3d ret = new Matrix3d();
      getRotation(ret);

      return ret;
   }

   public FramePlane3d getFace(FaceName faceName)
   {
      return new FramePlane3d(referenceFrame, box3d.getFace(faceName));
   }

   public void setTransform(RigidBodyTransform transform3D)
   {
      box3d.setTransform(transform3D);
   }

   public FrameBox3d(FrameBox3d other)
   {
      this(other.referenceFrame, other.box3d);
   }

   public FrameBox3d(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.box3d = new Box3d();
   }

   public FrameBox3d(ReferenceFrame referenceFrame, Box3d box3d)
   {
      this.referenceFrame = referenceFrame;
      this.box3d = new Box3d(box3d);
   }

   public FrameBox3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double lengthX, double widthY, double heightZ)
   {
      this.referenceFrame = referenceFrame;
      this.box3d = new Box3d(configuration, lengthX, widthY, heightZ);
   }

   public FrameBox3d(ReferenceFrame referenceFrame, double lengthX, double widthY, double heightZ)
   {
      this.referenceFrame = referenceFrame;
      this.box3d = new Box3d(lengthX, widthY, heightZ);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame != referenceFrame)
      {
         referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
         box3d.applyTransform(temporaryTransformToDesiredFrame);
         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   @Override
   public double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);

      return box3d.distance(point.getPoint());
   }

   @Override
   public void orthogonalProjection(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      box3d.orthogonalProjection(point.getPoint());
   }

   @Override
   public void applyTransform(RigidBodyTransform transformation)
   {
      box3d.applyTransform(transformation);
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest)
   {
      checkReferenceFrameMatch(pointToTest);

      return box3d.isInsideOrOnSurface(pointToTest.getPoint());
   }

   @Override
   public boolean isInsideOrOnSurface(FramePoint pointToTest, double epsilon)
   {
      checkReferenceFrameMatch(pointToTest);

      return box3d.isInsideOrOnSurface(pointToTest.getPoint(), epsilon);
   }

   @Override
   public void getClosestPointAndNormalAt(FramePoint intersectionToPack, FrameVector normalToPack, FramePoint pointToCheck)
   { // Assumes the point is inside. Otherwise, it doesn't really matter.
      checkReferenceFrameMatch(pointToCheck);
      normalToPack.changeFrame(referenceFrame);
      intersectionToPack.changeFrame(referenceFrame);
      box3d.checkIfInside(pointToCheck.getPoint(), intersectionToPack.getPoint(), normalToPack.getVector());
   }

   public void setAndChangeFrame(FrameBox3d other)
   {
      this.referenceFrame = other.referenceFrame;
      this.box3d.set(other.box3d);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(box3d.toString());

      return builder.toString();
   }

   public void packFramePose(FramePose framePoseToPack)
   {
      framePoseToPack.setPoseIncludingFrame(referenceFrame, box3d.transform);
   }

   public void scale(double scale)
   {
      box3d.scale(scale);
   }

   public void computeVertices(Point3d[] vertices)
   {
      box3d.computeVertices(vertices);
   }
}
