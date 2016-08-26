package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.interfaces.VectorInterface;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * One of the main goals of this class is to check, at runtime, that operations on vectors occur within the same Frame.
 * This method checks for one Vector argument.
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class FrameVector extends FrameTuple<FrameVector, TransformableVector3d> implements VectorInterface
{
   private static final long serialVersionUID = -4475317718392284548L;

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, Tuple3d tuple)
   {
      super(referenceFrame, new TransformableVector3d(tuple), null);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, Tuple3d tuple, String name)
   {
      super(referenceFrame, new TransformableVector3d(tuple), name);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, double[] vector)
   {
      super(referenceFrame, new TransformableVector3d(vector), null);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, double[] vector, String name)
   {
      super(referenceFrame, new TransformableVector3d(vector), name);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new TransformableVector3d(), null);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, String name)
   {
      super(referenceFrame, new TransformableVector3d(), name);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(FrameTuple<?, ?> frameTuple)
   {
      super(frameTuple.referenceFrame, new TransformableVector3d(frameTuple.tuple), frameTuple.name);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this(referenceFrame, x, y, z, null);
   }

   /** FrameVector <p/> A normal vector associated with a specific reference frame. */
   public FrameVector(ReferenceFrame referenceFrame, double x, double y, double z, String name)
   {
      super(referenceFrame, new TransformableVector3d(x, y, z), name);
   }

   public static FrameVector generateRandomFrameVector(Random random, ReferenceFrame frame)
   {
      FrameVector randomVector = new FrameVector(frame, RandomTools.generateRandomVector(random));
      return randomVector;
   }

   public static FrameVector generateRandomFrameVector(Random random, ReferenceFrame frame, double xMin, double xMax, double yMin, double yMax, double zMin,
         double zMax)
   {
      FrameVector randomVector = new FrameVector(frame, RandomTools.generateRandomDouble(random, xMin, xMax), RandomTools
            .generateRandomDouble(random, yMin, yMax),
            RandomTools.generateRandomDouble(random, zMin, zMax));
      return randomVector;
   }

   /**
    * Retrieves the vector inside this FrameVector
    *
    * @return Vector3d
    */
   public Vector3d getVector()
   {
      return this.tuple;
   }

   /**
    * Creates a new FrameVector2d based on the x and y components of this FrameVector
    */
   public FrameVector2d toFrameVector2d()
   {
      return new FrameVector2d(this.getReferenceFrame(), this.getX(), this.getY());
   }

   public double dot(Vector3d vector)
   {
      return this.tuple.dot(vector);
   }

   public double dot(FrameVector frameVector)
   {
      checkReferenceFrameMatch(frameVector);

      return this.tuple.dot(frameVector.tuple);
   }

   public double angle(Vector3d vector)
   {
      return this.tuple.angle(vector);
   }

   public double angle(FrameVector frameVector)
   {
      checkReferenceFrameMatch(frameVector);

      return this.tuple.angle(frameVector.tuple);
   }
   
   public boolean isEpsilonParallel(FrameVector frameVector, double epsilonAngle)
   {
      checkReferenceFrameMatch(frameVector);
      
      double angleMinusZeroToPi = Math.abs(AngleTools.trimAngleMinusPiToPi(this.angle(frameVector)));

      double errorFromParallel = Math.min(angleMinusZeroToPi, Math.PI - angleMinusZeroToPi);
      return errorFromParallel < epsilonAngle;
   }
   
   public boolean isEpsilonParallel(FrameVector frameVector)
   {
      return isEpsilonParallel(frameVector, 1e-7);
   }

   public void cross(FrameTuple<?, ?> frameTuple1)
   {
      checkReferenceFrameMatch(frameTuple1);
      cross(this.tuple, this.tuple, frameTuple1.tuple);
   }

   public void cross(Vector3d vector)
   {
      cross(this.tuple, this.tuple, vector);
   }

   public void cross(Vector3d tuple1, Vector3d tuple2)
   {
      cross(this.tuple, tuple1, tuple2);
   }

   public void cross(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      cross(this.tuple, frameTuple1.tuple, frameTuple2.tuple);
   }

   protected static final void cross(Vector3d resultToPack, Tuple3d t1, Tuple3d t2)
   {
      double x, y;

      x = t1.getY() * t2.getZ() - t1.getZ() * t2.getY();
      y = t2.getX() * t1.getZ() - t2.getZ() * t1.getX();
      resultToPack.setZ(t1.getX() * t2.getY() - t1.getY() * t2.getX());
      resultToPack.setX(x);
      resultToPack.setY(y);
   }

   public void normalize()
   {
      this.tuple.normalize();
   }

   public double length()
   {
      return tuple.length();
   }

   public double lengthSquared()
   {
      return tuple.lengthSquared();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(this.tuple);
   }
 
   @Override
   public void getVector(Vector3d vectorToPack)
   {
      this.get(vectorToPack);
   }

   @Override
   public void setVector(VectorInterface vectorInterface)
   {
      vectorInterface.getVector(this.getVector());
   }

   @Override
   public void setVector(Vector3d vector)
   {
      this.set(vector);
   }
}
