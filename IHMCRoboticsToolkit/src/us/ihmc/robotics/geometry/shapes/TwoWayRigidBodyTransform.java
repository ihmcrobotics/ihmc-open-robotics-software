package us.ihmc.robotics.geometry.shapes;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.Transformable;

/**
 * Class that by default, lazily keeps an inverse transform around.
 */
class TwoWayRigidBodyTransform
{
   private final RigidBodyTransform forwardTransform;
   private final RigidBodyTransform backwardTransform;
   
   private boolean forwardTransformIsOutOfDate;
   private boolean backwardTransformIsOutOfDate;
   
   public TwoWayRigidBodyTransform()
   {
      forwardTransform = new RigidBodyTransform();
      backwardTransform = new RigidBodyTransform();
      
      forwardTransformIsOutOfDate = false;
      backwardTransformIsOutOfDate = false;
   }
   
   public void setForwardTransform(RigidBodyTransform forwardTransform)
   {
      this.forwardTransform.set(forwardTransform);
      backwardTransformIsOutOfDate = true;
   }
   
   public void setBackwardTransform(RigidBodyTransform backwardTransform)
   {
      this.backwardTransform.set(backwardTransform);
      forwardTransformIsOutOfDate = true;
   }
   
   public RigidBodyTransform getForwardTransformUnsafe()
   {
      ensureForwardTransformUpToDate();
      return forwardTransform;
   }
   
   public RigidBodyTransform getBackwardTransformUnsafe()
   {
      ensureBackwardTransformUpToDate();
      return backwardTransform;
   }
   
   public void ensureTransformsUpToDate()
   {
      ensureForwardTransformUpToDate();
      ensureBackwardTransformUpToDate();
   }
   
   private void ensureForwardTransformUpToDate()
   {
      if (forwardTransformIsOutOfDate)
      {
         forwardTransform.set(backwardTransform);
         forwardTransform.invert();
      }
   }
   
   private void ensureBackwardTransformUpToDate()
   {
      if (backwardTransformIsOutOfDate)
      {
         backwardTransform.set(forwardTransform);
         backwardTransform.invert();
      }
   }
   
   public void transformForward(Transformable transformable)
   {
      ensureForwardTransformUpToDate();
      transformable.applyTransform(forwardTransform);
   }

   public void transformForward(Vector3f vector)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vector);
   }

   public void transformForward(Vector4f vector)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vector);
   }

   public void transformForward(Vector3d vector)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vector);
   }

   public void transformForward(Vector4d vector)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vector);
   }

   public void transformForward(Vector3f vectorIn, Vector3f vectorOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformForward(Vector4f vectorIn, Vector4f vectorOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformForward(Vector3d vectorIn, Vector3d vectorOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformForward(Vector4d vectorIn, Vector4d vectorOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vectorIn, vectorOut);
   }

   public void transformForward(Point3f point)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(point);
   }
   
   public void transformForward(Point3d point)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(point);
   }
   
   public void transformForward(Point3f pointIn, Point3f pointOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(pointIn, pointOut);
   }
   
   public void transformForward(Point3d pointIn, Point3d pointOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(pointIn, pointOut);
   }
   
   public void transformBackward(Transformable transformable)
   {
      ensureBackwardTransformUpToDate();
      transformable.applyTransform(backwardTransform);
   }

   public void transformBackward(Vector3f vector)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vector);
   }

   public void transformBackward(Vector4f vector)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vector);
   }

   public void transformBackward(Vector3d vector)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vector);
   }

   public void transformBackward(Vector4d vector)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vector);
   }

   public void transformBackward(Vector3f vectorIn, Vector3f vectorOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformBackward(Vector4f vectorIn, Vector4f vectorOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformBackward(Vector3d vectorIn, Vector3d vectorOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformBackward(Vector4d vectorIn, Vector4d vectorOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vectorIn, vectorOut);
   }

   public void transformBackward(Point3f point)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(point);
   }
   
   public void transformBackward(Point3d point)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(point);
   }
   
   public void transformBackward(Point3f pointIn, Point3f pointOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(pointIn, pointOut);
   }
   
   public void transformBackward(Point3d pointIn, Point3d pointOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(pointIn, pointOut);
   }
}
