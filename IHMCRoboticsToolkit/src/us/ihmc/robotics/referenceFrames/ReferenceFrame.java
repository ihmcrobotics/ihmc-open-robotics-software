package us.ihmc.robotics.referenceFrames;

import java.io.Serializable;
import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeHolder;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;

/**
 * <p>
 * ReferenceFrame
 * </p>
 *
 * <p>
 * Description: ReferenceFrame is used to represent a reference coordinate frame. One constructor
 * allows the creation of a "root" frame with no parent. The other creates a child frame that has a
 * reference and transform to a parent. HumanoidReferenceFrames are used in classes like FramePoint
 * to indicate which frame the point is defined in.
 * </p>
 *
 */
public abstract class ReferenceFrame implements Serializable, NameBasedHashCodeHolder
{
   private static final long serialVersionUID = 9129810880579453658L;
   /** The name of this reference frame. The name should preferably be unique. */
   private final String frameName;
   /**
    * A secondary unique hash code representing this reference frame that is computed based on
    * {@link #frameName} and the parent frame name if any.
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same tree of reference frames, such that it can be used to serialize and deserialize frame
    * information.
    * </p>
    */
   private final long nameBasedHashCode;
   /**
    * Additional custom hash code representing this frame.
    * <p>
    * Somewhat of a hack that allows to enforce two frames that are physically the same but with
    * different names to have the same hash code or to enforce a common frame to have a specific
    * hash code that can be known without holding on its actual instance.
    * </p>
    */
   private long additionalNameBasedHashCode;
   /**
    * The reference to which this frame is attached to.
    * <p>
    * The {@link #transformToParent} of this describes the pose of this reference frame with respect
    * to {@link #parentFrame}.
    * </p>
    */
   protected final ReferenceFrame parentFrame;
   /**
    * Entire from the root frame to this used to efficiently compute the pose of this reference
    * frame with respect to the root frame.
    */
   private final ReferenceFrame[] framesStartingWithRootEndingWithThis;

   /**
    * The pose of this transform with respect to its parent.
    * <p>
    * Notes:
    * <ul>
    * The root frame has no parent such that {@code transformToParent == null}.
    * <ul>
    * The transform can be constant over time or can change depending on the final implementation of
    * {@code ReferenceFrame}.
    * </p>
    */
   private final RigidBodyTransform transformToParent;

   // These need to be longs instead of integers or they'll role over too soon. With longs, you get at least 100 years of runtime.
   protected static long nextTransformToRootID = 1;
   private long transformToRootID = Long.MIN_VALUE;
   /**
    * The current transform from this reference frame to the root frame.
    * <p>
    * For instance, one can calculate the coordinates in the root frame P<sub>root</sub> of a point
    * P expressed in this frame as follows:<br>
    * {@code transformToRoot.transform}(P, P<sub>root</sub>)
    * </p>
    */
   private final RigidBodyTransform transformToRoot;
   /**
    * The current transform from the root frame to this reference frame.
    * <p>
    * For instance, one can calculate the coordinates in this reference frame P<sub>this</sub> of a
    * point P expressed in the root frame as follows:<br>
    * {@code inverseTransformToRoot.transform}(P, P<sub>this</sub>)
    * </p>
    */
   private final RigidBodyTransform inverseTransformToRoot;

   /**
    * Field initialized at construction time that specifies if this reference frame represents a
    * stationary frame, i.e. a non-moving frame, with respect to the root reference frame.
    */
   private final boolean isAStationaryFrame; // TODO when isAStationaryFrame == true, transformToParent should be immutable.
   /**
    * Field initialized at construction time that specifies if at all time the z-axis of this
    * reference frame remains aligned with the z-axis of the root frame.
    */
   private final boolean isZupFrame;

   /**
    * {@code worldFrame} is an inertial root reference frame with its axes aligned as follows:
    * <ul>
    * <li>The x-axis is usually referred to as the forward axis.
    * <li>With the x-axis referring forward, the y-axis points to the left.
    * <li>The z-axis points upward and usually points to the opposite direction to the gravitational
    * acceleration.
    * </ul>
    */
   private static final ReferenceFrame worldFrame = constructARootFrame("World");

   /**
    * Construct a new inertial z-up root reference frame.
    * <p>
    * Most of the time, {@link #worldFrame} is the only root frame from which children
    * reference frames are added.
    * </p>
    * <p>
    * Note that frames added as children of this root frame belongs to a different reference frame
    * tree than the tree starting off of {@link #worldFrame}. Transformation across two different
    * trees of reference frames is forbidden as the transformation between them is undefined.
    * </p>
    * 
    * @param frameName the name of the new world frame.
    * @return the new non-moving z-up root reference frame.
    */
   public static ReferenceFrame constructARootFrame(String frameName)
   {
      ReferenceFrame ret = new ReferenceFrame(frameName, true, true)
      {
         private static final long serialVersionUID = -8828178814213025690L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      return ret;
   }

   /**
    * Creates a new reference frame such that it is centered at the given {@code point} and with its
    * z-axis aligned with the given {@code zAxis} vector.
    * <p>
    * Note that the parent frame is set to the reference frame the given {@code point} and
    * {@code zAxis} are expressed in.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param point location of the reference frame's origin. Not modified.
    * @param zAxis orientation the reference frame's z-axis. Not modified.
    * @return the new reference frame.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code zAxis} are not expressed
    *            in the same reference frame.
    */
   public static ReferenceFrame constructReferenceFrameFromPointAndZAxis(String frameName, FramePoint point, FrameVector zAxis)
   {
      point.checkReferenceFrameMatch(zAxis.getReferenceFrame());

      RigidBodyTransform transformToParent = TransformTools.createTransformFromPointAndZAxis(point, zAxis);

      return constructFrameWithUnchangingTransformToParent(frameName, point.getReferenceFrame(), transformToParent);
   }

   /**
    * Creates a new reference frame such that it is centered at the given {@code point} and with one
    * of its axes aligned with the given {@code alignAxisWithThis} vector.
    * <p>
    * Note that the parent frame is set to the reference frame the given {@code point} and
    * {@code alignAxisWithThis} are expressed in.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param point location of the reference frame's origin. Not modified.
    * @param axisToAlign defines which axis of the new reference frame is to be aligned with the
    *           given {@code alignAxisWithThis} vector.
    * @param alignAxisWithThis the vector to which the reference frame chosen axis should be aligned
    *           with. Not modified.
    * @return the new reference frame.
    * @throws ReferenceFrameMismatchException if {@code point} and {@code alignAxisWithThis} are not
    *            expressed in the same reference frame.
    */
   public static ReferenceFrame constructReferenceFrameFromPointAndAxis(String frameName, FramePoint point, Axis axisToAlign, FrameVector alignAxisWithThis)
   {
      point.checkReferenceFrameMatch(alignAxisWithThis.getReferenceFrame());

      FrameVector referenceNormal = new FrameVector();

      switch (axisToAlign)
      {
      case X:
         referenceNormal.setIncludingFrame(point.getReferenceFrame(), 1.0, 0.0, 0.0);
         break;
      case Y:
         referenceNormal.setIncludingFrame(point.getReferenceFrame(), 0.0, 1.0, 0.0);
         break;
      case Z:
         referenceNormal.setIncludingFrame(point.getReferenceFrame(), 0.0, 0.0, 1.0);
         break;

      default:
         break;
      }

      AxisAngle rotationToDesired = new AxisAngle();
      alignAxisWithThis.changeFrame(referenceNormal.getReferenceFrame());
      EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(referenceNormal.getVectorCopy(), alignAxisWithThis.getVectorCopy(), rotationToDesired);

      RigidBodyTransform transformToDesired = new RigidBodyTransform();

      transformToDesired.setRotation(rotationToDesired);
      transformToDesired.setTranslation(point.getPoint());

      return constructFrameWithUnchangingTransformToParent(frameName, point.getReferenceFrame(), transformToDesired);
   }

   /**
    * Generates a reference frame with a random transform to its parent frame.
    * <p>
    * This is usually used for test purposes.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      RigidBodyTransform transformFromParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      return constructFrameWithUnchangingTransformFromParent(frameName, parentFrame, transformFromParent);
   }

   /**
    * Creates a reference frame with an immutable transform from its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the parent frame expressed in this
    * frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param transformFromParent the transform that can be used to transform a geometry object from
    *           the parent frame to this frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithUnchangingTransformFromParent(String frameName, ReferenceFrame parentFrame,
                                                                                RigidBodyTransform transformFromParent)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform(transformFromParent);
      transformToParent.invert();

      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);
   }

   /**
    * Creates a reference frame with an immutable translation offset from its parent.
    * <p>
    * The new reference frame has the same orientation as its parent frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param translationOffsetFromParent describes the position of the new reference frame's origin
    *           expressed in the parent frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithUnchangingTranslationFromParent(String frameName, ReferenceFrame parentFrame,
                                                                                  Tuple3DReadOnly translationOffsetFromParent)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslation(translationOffsetFromParent);

      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent, parentFrame.isAStationaryFrame, parentFrame.isZupFrame);
   }

   /**
    * Creates a reference frame with an immutable transform from its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the parent frame expressed in this
    * frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param transformFromParent the transform that can be used to transform a geometry object from
    *           the parent frame to this frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructBodyFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame,
                                                                                  RigidBodyTransform transformToParent)
   {
      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent, false, false);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame,
                                                                              RigidBodyTransform transformToParent)
   {
      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent, false, false);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame,
                                                                              RigidBodyTransform transformToParent, boolean isWorldFrame, boolean isZupFrame)
   {
      //      if (!RotationFunctions.isRotationProper(transformToParent))
      //         throw new RuntimeException("Rotation not normalized: " + transformToParent);
      ReferenceFrame ret = new ReferenceFrame(frameName, parentFrame, isWorldFrame, isZupFrame)
      {
         private static final long serialVersionUID = 4694374344134623529L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      ret.transformToParent.set(transformToParent);

      return ret;
   }

   /**
    * Return the world reference frame that is a root reference frame. If located at (0, 0, 0) and
    * its coordinate system is such as the z-axis is up, x-axis is forward, and y-axis points to the
    * left.
    */
   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public ReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this.frameName = frameName;
      this.parentFrame = parentFrame;
      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(frameName, parentFrame.getName());
      this.framesStartingWithRootEndingWithThis = constructFramesStartingWithRootEndingWithThis(this);

      this.transformToRoot = new RigidBodyTransform();
      this.inverseTransformToRoot = new RigidBodyTransform();
      this.transformToParent = new RigidBodyTransform();

      this.isAStationaryFrame = false;
      this.isZupFrame = false;
   }

   /**
    * This constructor creates a "top level" reference frame with the specified name. The parent
    * frame and transforms are null.
    *
    * @param frameName String
    */
   public ReferenceFrame(String frameName, boolean isWorldFrame, boolean isZupFrame)
   {
      this.frameName = frameName;
      nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(frameName);
      this.parentFrame = null;
      this.transformToRootID = 0;

      this.framesStartingWithRootEndingWithThis = new ReferenceFrame[] {this};

      this.transformToRoot = null;
      this.inverseTransformToRoot = null;
      this.transformToParent = null;

      this.isAStationaryFrame = isWorldFrame;
      this.isZupFrame = isZupFrame;
   }

   /**
    * This constructor creates a child reference frame with the specified name. The parent frame can
    * be a "top level" frame or another child. The Transform defines how to convert a vector from
    * the parent frame to the child frame. An inverse transform is created automatically based on
    * the provided frame.
    *
    * @param frameName String
    * @param parentFrame Frame
    * @param transformToParent Transform3D
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isWorldFrame, boolean isZupFrame)
   {
      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(frameName, parentFrame.getName());
      this.frameName = frameName;
      this.parentFrame = parentFrame;
      this.framesStartingWithRootEndingWithThis = constructFramesStartingWithRootEndingWithThis(this);

      this.transformToRoot = new RigidBodyTransform();
      this.inverseTransformToRoot = new RigidBodyTransform();

      this.transformToParent = new RigidBodyTransform(transformToParent);
      this.transformToParent.normalizeRotationPart();

      this.isAStationaryFrame = isWorldFrame;
      this.isZupFrame = isZupFrame;
   }

   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isWorldFrame, boolean isZupFrame)
   {
      this.frameName = frameName;
      this.parentFrame = parentFrame;
      this.framesStartingWithRootEndingWithThis = constructFramesStartingWithRootEndingWithThis(this);

      this.isAStationaryFrame = isWorldFrame;
      this.isZupFrame = isZupFrame;

      this.transformToRoot = new RigidBodyTransform();
      this.inverseTransformToRoot = new RigidBodyTransform();
      this.transformToParent = new RigidBodyTransform();

      if (parentFrame != null)
      {
         nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(frameName, parentFrame.getName());
      }
      else
      {
         nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(frameName);
      }
   }

   private static ReferenceFrame[] constructFramesStartingWithRootEndingWithThis(ReferenceFrame thisFrame)
   {
      ReferenceFrame parentFrame = thisFrame.parentFrame;
      if (parentFrame == null)
      {
         return new ReferenceFrame[] {thisFrame};
      }

      int size = parentFrame.framesStartingWithRootEndingWithThis.length + 1;
      ReferenceFrame[] ret = new ReferenceFrame[size];

      for (int i = 0; i < size - 1; i++)
      {
         ret[i] = parentFrame.framesStartingWithRootEndingWithThis[i];
      }

      ret[size - 1] = thisFrame;

      return ret;
   }

   public boolean isWorldFrame()
   {
      return isAStationaryFrame;
   }

   public boolean isZupFrame()
   {
      return isZupFrame;
   }

   /**
    * The user must call update each tick. It will then call updateTransformToParent.
    */
   public void update()
   {
      if (parentFrame == null)
      {
         return;
      }

      updateTransformToParent(transformToParent);

      //      transformToParent.normalize();
      transformToRootID = Long.MIN_VALUE;
   }

   private RigidBodyTransform preCorruptionTransform, postCorruptionTransform;

   public void corruptTransformToParentPreMultiply(RigidBodyTransform preCorruptionTransform)
   {
      if (this.preCorruptionTransform == null)
      {
         this.preCorruptionTransform = new RigidBodyTransform();
      }

      this.preCorruptionTransform.set(preCorruptionTransform);
      this.update();
   }

   public void corruptTransformToParentPostMultiply(RigidBodyTransform postCorruptionTransform)
   {
      if (this.postCorruptionTransform == null)
      {
         this.postCorruptionTransform = new RigidBodyTransform();
      }

      this.postCorruptionTransform.set(postCorruptionTransform);
      this.update();
   }

   protected abstract void updateTransformToParent(RigidBodyTransform transformToParent);

   /**
    * getParent
    *
    * Returns the parent frame of this reference frame
    *
    * @return Frame
    */
   public ReferenceFrame getParent()
   {
      return parentFrame;
   }

   public ReferenceFrame getRootFrame()
   {
      return framesStartingWithRootEndingWithThis[0];
   }

   /**
    * getTransformToParent
    *
    * Returns a Transform3D that can be applied to a vector defined in this frame in order to obtain
    * the equivalent vector in the parent frame
    *
    * @return Transform3D
    */
   public RigidBodyTransform getTransformToParent()
   {
      return new RigidBodyTransform(transformToParent);
   }

   public void getTransformToParent(RigidBodyTransform transformToPack)
   {
      updateTransformToParent(transformToPack);
   }

   public String getName()
   {
      return frameName;
   }

   /**
    * @deprecated Creates garbage without warning. - dcalvert
    */
   public RigidBodyTransform getTransformToDesiredFrame(ReferenceFrame desiredFrame)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, desiredFrame);

      return ret;
   }

   /**
    * @deprecated Creates garbage without warning. - dcalvert
    */
   public RigidBodyTransform getTransformToWorldFrame()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, worldFrame);
      return ret;
   }

   // JEP 10117: Temporary transform to save lots of memory generation.
   // Determined it was needed after running profiler.
   // JEP 101215: Made it a serializableTransform3D to make sure LittleDog planner RMI stuff still works.

   public void getTransformToDesiredFrame(RigidBodyTransform transformToPack, ReferenceFrame desiredFrame)
   {
      verifySameRoots(desiredFrame);

      this.efficientComputeTransform();
      desiredFrame.efficientComputeTransform();

      if (desiredFrame.inverseTransformToRoot != null)
      {
         if (this.transformToRoot != null)
         {
            transformToPack.set(desiredFrame.inverseTransformToRoot);
            transformToPack.multiply(this.transformToRoot);
         }
         else
         {
            transformToPack.set(desiredFrame.inverseTransformToRoot);
         }
      }
      else
      {
         if (this.transformToRoot != null)
         {
            transformToPack.set(this.transformToRoot);
         }
         else
         {
            transformToPack.setIdentity();
         }
      }
   }

   public boolean isParentFrame(ReferenceFrame frame)
   {
      return frame == parentFrame;
   }

   public boolean isChildFrame(ReferenceFrame frame)
   {
      return frame.isParentFrame(this);
   }

   public void verifySameRoots(ReferenceFrame referenceFrame)
   {
      if (this.getRootFrame() != referenceFrame.getRootFrame())
      {
         throw new RuntimeException("Frames do not have same roots. this = " + this + ", referenceFrame = " + referenceFrame);
      }
   }

   public RigidBodyTransform getTransformToRoot()
   {
      efficientComputeTransform();
      return transformToRoot;
   }

   public RigidBodyTransform getInverseTransformToRoot()
   {
      efficientComputeTransform();
      return inverseTransformToRoot;
   }

   private void efficientComputeTransform()
   {
      int chainLength = this.framesStartingWithRootEndingWithThis.length;

      boolean updateFromHereOnOut = false;
      long previousUpdateId = 0;

      for (int i = 0; i < chainLength; i++)
      {
         ReferenceFrame referenceFrame = framesStartingWithRootEndingWithThis[i];

         if (!updateFromHereOnOut)
         {
            if (referenceFrame.transformToRootID < previousUpdateId)
            {
               updateFromHereOnOut = true;
               nextTransformToRootID++;
            }
         }

         if (updateFromHereOnOut)
         {
            if (referenceFrame.parentFrame != null)
            {
               RigidBodyTransform parentsTransformToRoot = referenceFrame.parentFrame.transformToRoot;
               if (parentsTransformToRoot != null)
               {
                  referenceFrame.transformToRoot.set(parentsTransformToRoot);
               }
               else
               {
                  referenceFrame.transformToRoot.setIdentity();
               }

               if (referenceFrame.preCorruptionTransform != null)
               {
                  referenceFrame.transformToRoot.multiply(referenceFrame.preCorruptionTransform);
               }

               referenceFrame.transformToRoot.multiply(referenceFrame.transformToParent);

               if (referenceFrame.postCorruptionTransform != null)
               {
                  referenceFrame.transformToRoot.multiply(referenceFrame.postCorruptionTransform);
               }

               referenceFrame.transformToRoot.normalizeRotationPart();
               referenceFrame.inverseTransformToRoot.setAndInvert(referenceFrame.transformToRoot);

               referenceFrame.transformToRootID = nextTransformToRootID;
            }
         }

         previousUpdateId = referenceFrame.transformToRootID;
      }
   }

   @Override
   public String toString()
   {
      return frameName; // + "\nTransform to Parent = " + this.transformToParent;
   }

   public void checkReferenceFrameMatch(AbstractReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   public void checkReferenceFrameMatch(ReferenceFrame referenceFrame) throws ReferenceFrameMismatchException
   {
      if (this != referenceFrame)
      {
         String msg = "Argument's frame " + referenceFrame + " does not match " + this;

         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public void checkIsWorldFrame() throws RuntimeException
   {
      if (!isWorldFrame())
      {
         throw new RuntimeException("Frame " + this + " is not a world frame");
      }
   }

   void checkRepInvariants()
   {
      if (framesStartingWithRootEndingWithThis[framesStartingWithRootEndingWithThis.length - 1] != this)
      {
         throw new RuntimeException("This must be the last frame in the chain.");
      }

      if (parentFrame == null)
      {
         if (framesStartingWithRootEndingWithThis.length != 1)
         {
            throw new RuntimeException("If the parentFrame is null, then this must be a root frame, in which there should be only one frame in the chain.");
         }

         if (transformToParent != null)
         {
            throw new RuntimeException("Root frames don't have transformToParent or transformToRoot defined. This is so RMI still works with frames since Transform3D is not serializable.");
         }

         if (transformToRoot != null)
         {
            throw new RuntimeException("Root frames don't have transformToParent or transformToRoot defined. This is so RMI still works with frames since Transform3D is not serializable.");
         }

         if (this.transformToRootID != 0)
         {
            System.err.println("this ReferenceFrame = " + this);

            throw new RuntimeException("transformToRootID = " + transformToRootID + ", Root frames must not be updated.");
         }
      }
      else
      {
         if (framesStartingWithRootEndingWithThis[framesStartingWithRootEndingWithThis.length - 2] != parentFrame)
         {
            throw new RuntimeException("The parent must be the second to last frame in the chain.");
         }

         long maxIdSoFar = 0;
         RigidBodyTransform computedTransformToRoot = new RigidBodyTransform();
         for (int i = 1; i < framesStartingWithRootEndingWithThis.length; i++)
         {
            ReferenceFrame frame = framesStartingWithRootEndingWithThis[i];
            computedTransformToRoot.multiply(frame.transformToParent);

            long id = frame.transformToRootID;
            if (id < maxIdSoFar)
            {
               // Only need to make sure things are consistent down to where the
               break;
            }

            maxIdSoFar = id;

            if (!frame.transformToRoot.epsilonEquals(computedTransformToRoot, 1e-5))
            {
               System.err.println("frame.transformToRoot = " + frame.transformToRoot + ", computedTransformToRoot = " + computedTransformToRoot);
               System.err.println("this = " + this + " frame = " + frame);

               throw new RuntimeException("transformToRoot is inconsistent!!");
            }
         }
      }
   }

   @Override
   public long getNameBasedHashCode()
   {
      return nameBasedHashCode;
   }

   public long getAdditionalNameBasedHashCode()
   {
      return additionalNameBasedHashCode;
   }

   public void setAdditionalNameBasedHashCode(long additionalNameBasedHashCode)
   {
      this.additionalNameBasedHashCode = additionalNameBasedHashCode;
   }
}
