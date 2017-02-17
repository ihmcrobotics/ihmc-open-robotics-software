package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Class that by default, lazily keeps an inverse transform around.
 */
class TwoWayRigidBodyTransform
{
   private final RigidBodyTransform forwardTransform;
   private final RigidBodyTransform backwardTransform;
   
   private boolean forwardTransformIsOutOfDate = false;
   private boolean backwardTransformIsOutOfDate = false;
   
   public TwoWayRigidBodyTransform()
   {
      forwardTransform = new RigidBodyTransform();
      backwardTransform = new RigidBodyTransform();
   }
   
   public void setForwardTransform(RigidBodyTransform forwardTransform)
   {
      this.forwardTransform.set(forwardTransform);
      forwardTransformIsOutOfDate = false;
      backwardTransformIsOutOfDate = true;
   }
   
   public void setBackwardTransform(RigidBodyTransform backwardTransform)
   {
      this.backwardTransform.set(backwardTransform);
      backwardTransformIsOutOfDate = false;
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
         forwardTransformIsOutOfDate = false;
      }
   }
   
   private void ensureBackwardTransformUpToDate()
   {
      if (backwardTransformIsOutOfDate)
      {
         backwardTransform.set(forwardTransform);
         backwardTransform.invert();
         backwardTransformIsOutOfDate = false;
      }
   }

   public boolean isForwardTransformOutOfDate()
   {
      return forwardTransformIsOutOfDate;
   }

   public boolean isBackwardTransformOutOfDate()
   {
      return backwardTransformIsOutOfDate;
   }

   public void transformForward(Transformable transformable)
   {
      ensureForwardTransformUpToDate();
      transformable.applyTransform(forwardTransform);
   }

   public void transformForward(Vector3DReadOnly vectorIn, Vector3DBasics vectorOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformForward(Vector4DReadOnly vectorIn, Vector4DBasics vectorOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(vectorIn, vectorOut);
   }

   public void transformForward(Point3DReadOnly pointIn, Point3DBasics pointOut)
   {
      ensureForwardTransformUpToDate();
      forwardTransform.transform(pointIn, pointOut);
   }
   
   public void transformBackward(Transformable transformable)
   {
      ensureBackwardTransformUpToDate();
      transformable.applyTransform(backwardTransform);
   }

   public void transformBackward(Vector3DReadOnly vectorIn, Vector3DBasics vectorOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformBackward(Vector4DReadOnly vectorIn, Vector4DBasics vectorOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(vectorIn, vectorOut);
   }
   
   public void transformBackward(Point3DReadOnly pointIn, Point3DBasics pointOut)
   {
      ensureBackwardTransformUpToDate();
      backwardTransform.transform(pointIn, pointOut);
   }
}
