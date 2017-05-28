package us.ihmc.robotics.referenceFrames;

import java.io.Serializable;
import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
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
    * {@code worldFrame} is a root reference frame and is most of time the only root reference
    * frame.
    * <p>
    * It is commonly assumed that its axes are aligned as follows:
    * <ul>
    * <li>The x-axis is usually referred to as the forward axis.
    * <li>With the x-axis referring forward, the y-axis points to the left.
    * <li>The z-axis points upward and usually points to the opposite direction to the gravitational
    * acceleration.
    * </ul>
    * </p>
    */
   private static final ReferenceFrame worldFrame = constructARootFrame("World");

   /**
    * Construct a new z-up root reference frame.
    * <p>
    * Most of the time, {@link #worldFrame} is the only root frame from which children reference
    * frames are added.
    * </p>
    * <p>
    * Note that frames added as children of this root frame belongs to a different reference frame
    * tree than the tree starting off of {@link #worldFrame}. Transformation across two different
    * trees of reference frames is forbidden as the transformation between them is undefined.
    * </p>
    * <p>
    * The parent frame and transforms of a root frame are all {@code null}.
    * </p>
    *
    * @param frameName the name of the new world frame.
    * @return the new non-moving z-up root reference frame.
    */
   public static ReferenceFrame constructARootFrame(String frameName)
   {
      ReferenceFrame ret = new ReferenceFrame(frameName)
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
    * new frame.
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

      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);
   }

   /**
    * Creates a reference frame with an immutable transform to its parent.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its
    * parent frame.
    * </p>
    *
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *           frame to its parent frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame,
                                                                              RigidBodyTransform transformToParent)
   {
      boolean isZupFrame = parentFrame.isZupFrame && transformToParent.isRotation2D();
      boolean isAStationaryFrame = parentFrame.isAStationaryFrame;

      ReferenceFrame ret = new ReferenceFrame(frameName, parentFrame, transformToParent, isAStationaryFrame, isZupFrame)
      {
         private static final long serialVersionUID = 4694374344134623529L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

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

   /**
    * Creates a new root reference frame.
    * <p>
    * Please use the method {@link #constructARootFrame(String)} instead. This is to use only when
    * extending this class.
    * </p>
    * <p>
    * Most of the time, {@link #worldFrame} is the only root frame from which children reference
    * frames are added.
    * </p>
    * <p>
    * Note that frames added as children of this root frame belongs to a different reference frame
    * tree than the tree starting off of {@link #worldFrame}. Transformation across two different
    * trees of reference frames is forbidden as the transformation between them is undefined.
    * </p>
    * <p>
    * The parent frame and transforms of a root frame are all {@code null}.
    * </p>
    *
    * @param frameName the name of the new world frame.
    */
   public ReferenceFrame(String frameName)
   {
      this(frameName, null, null, true, true);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame}.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding
    * it.
    * </p>
    * <p>
    * This new reference frame is not a stationary frame, i.e. it is assumed to be potentially
    * moving with respect to the root frame. It is also not expected to have its z-axis aligned at
    * all time with the z-axis of the root frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this(frameName, parentFrame, null, false, false);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame}.
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding
    * it.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param isAStationaryFrame refers to whether this new frame is stationary with respect to the
    *           root frame or moving. If {@code true}, the {@code parentFrame} has to also be
    *           stationary.
    * @param isZupFrame refers to whether this new frame has its z-axis aligned with the root frame
    *           at all time or not.
    * @throws IllegalArgumentException if {@code isAStationaryFrame} is {@code true} and the
    *            {@code parentFrame} is not a stationary frame.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isAStationaryFrame, boolean isZupFrame)
   {
      this(frameName, parentFrame, null, isAStationaryFrame, isZupFrame);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the new frame expressed in its
    * parent frame.
    * </p>
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding
    * it.
    * </p>
    * <p>
    * This new reference frame is not a stationary frame, i.e. it is assumed to be potentially
    * moving with respect to the root frame. It is also not expected to have its z-axis aligned at
    * all time with the z-axis of the root frame.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *           frame to its parent frame. Not modified.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      this(frameName, parentFrame, transformToParent, false, false);
   }

   /**
    * Creates a new reference frame defined as being a child of the given {@code parentFrame} and
    * initializes the transform to its parent.
    * <p>
    * The {@code transformFromParent} should describe the pose of the new frame expressed in its
    * parent frame.
    * </p>
    * <p>
    * This new reference frame defined in the {@code parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the {@code parentFrame} can be modified at runtime by changing the
    * transform in the method {@link #updateTransformToParent(RigidBodyTransform)} when overriding
    * it.
    * </p>
    * 
    * @param frameName the name of the new frame.
    * @param parentFrame the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *           frame to its parent frame. Not modified.
    * @param isAStationaryFrame refers to whether this new frame is stationary with respect to the
    *           root frame or moving. If {@code true}, the {@code parentFrame} has to also be
    *           stationary.
    * @param isZupFrame refers to whether this new frame has its z-axis aligned with the root frame
    *           at all time or not.
    * @throws IllegalArgumentException if {@code isAStationaryFrame} is {@code true} and the
    *            {@code parentFrame} is not a stationary frame.
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isAStationaryFrame, boolean isZupFrame)
   {
      this.frameName = frameName;
      this.parentFrame = parentFrame;
      framesStartingWithRootEndingWithThis = constructFramesStartingWithRootEndingWithThis(this);

      if (parentFrame == null)
      { // Setting up this ReferenceFrame as a root frame.
         transformToRootID = 0;
         nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(frameName);

         transformToRoot = null;
         inverseTransformToRoot = null;
         this.transformToParent = null;

         this.isAStationaryFrame = true;
         this.isZupFrame = true;
      }
      else
      {
         nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(frameName, parentFrame.getName());

         transformToRoot = new RigidBodyTransform();
         inverseTransformToRoot = new RigidBodyTransform();
         this.transformToParent = new RigidBodyTransform();

         if (transformToParent != null)
         {
            this.transformToParent.set(transformToParent);
            this.transformToParent.normalizeRotationPart();
         }

         if (isAStationaryFrame && !parentFrame.isAStationaryFrame)
            throw new IllegalArgumentException("The child of a non-stationary frame cannot be stationary.");

         this.isAStationaryFrame = isAStationaryFrame;
         this.isZupFrame = isZupFrame;
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

   /**
    * Tests if this reference frame is {@link #worldFrame}.
    * 
    * @return {@code true} if this is {@link #worldFrame}, {@code false} otherwise.
    */
   public boolean isWorldFrame()
   {
      return this == worldFrame;
   }

   /**
    * Tests if this reference frame is to be considered as stationary frame, i.e. not moving with
    * respect to its root frame.
    * 
    * @return {@code true} if this is a stationary frame, {@code false} other.
    */
   public boolean isAStationaryFrame()
   {
      return isAStationaryFrame;
   }

   /**
    * Tests if this reference frame is considered to have its z-axis aligned with the root frame.
    * 
    * @return {@code true} if this is a z-up frame, {@code false} otherwise.
    */
   public boolean isZupFrame()
   {
      return isZupFrame;
   }

   /**
    * The user must call update each tick. It will then call
    * {@link #updateTransformToParent(RigidBodyTransform)} which should be overridden to indicate
    * how the transform to each frame's parent should be updated.
    * <p>
    * Note that it is not necessary to call update on reference frames with an unchanging transform
    * to parent, even if the parent frame is moving.
    * </p>
    */
   public void update()
   {
      if (parentFrame == null)
      {
         return;
      }

      updateTransformToParent(transformToParent);
      transformToRootID = Long.MIN_VALUE;
   }

   /**
    * Override this method to define how this reference frame should be located with respect to its
    * parent frame over time by setting the argument {@code transformToParent}.
    * <p>
    * The {@code transformFromParent} should describe the pose of this frame expressed in its parent
    * frame.
    * </p>
    * 
    * @param transformToParent the transform to updated according to how this reference frame should
    *           now positioned with respect to its parent frame. Modified.
    */
   protected abstract void updateTransformToParent(RigidBodyTransform transformToParent);

   /**
    * Returns the parent frame of this reference frame.
    * <p>
    * Note that a root frame has no parent frame, such that this method returns {@code null} if this
    * is a root frame.
    * </p>
    *
    * @return the parent frame of this reference frame.
    */
   public ReferenceFrame getParent()
   {
      return parentFrame;
   }

   /**
    * Retrieves the root frame of the tree of reference frame that this frame belongs to.
    * 
    * @return the root frame.
    */
   public ReferenceFrame getRootFrame()
   {
      return framesStartingWithRootEndingWithThis[0];
   }

   /**
    * Returns a copy of this reference frame's transform to parent.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * This transform can be applied to a vector defined in this frame in order to obtain the
    * equivalent vector in the parent frame.
    * </p>
    *
    * @return a copy of the transform to the parent frame.
    */
   public RigidBodyTransform getTransformToParent()
   {
      return new RigidBodyTransform(transformToParent);
   }

   /**
    * packs this reference frame's transform to parent into the given transform
    * {@code transformToPack}.
    * <p>
    * This transform can be applied to a vector defined in this frame in order to obtain the
    * equivalent vector in the parent frame.
    * </p>
    *
    * @param transformToPack the transform in which this frame's transform to its parent frame is
    *           stored. Modified.
    */
   public void getTransformToParent(RigidBodyTransform transformToPack)
   {
      updateTransformToParent(transformToPack);
   }

   /**
    * Gets the name of this reference frame.
    * <p>
    * Reference frames usually have a unique name among the reference frames in the same tree but
    * this is not guaranteed.
    * </p>
    * 
    * @return this frame's name.
    */
   public String getName()
   {
      return frameName;
   }

   /**
    * Returns the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in the {@code desiredFrame}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param desiredFrame the goal frame.
    * @return the transform from this frame to the {@code desiredFrame}.
    */
   public RigidBodyTransform getTransformToDesiredFrame(ReferenceFrame desiredFrame)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, desiredFrame);

      return ret;
   }

   /**
    * Returns the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in {@link #worldFrame}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the transform from this frame to the {@link #worldFrame}.
    */
   public RigidBodyTransform getTransformToWorldFrame()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, worldFrame);
      return ret;
   }

   /**
    * Packs the transform that can be used to transform a geometry object defined in this frame to
    * obtain its equivalent expressed in the {@code desiredFrame} into {@code transformToPack}.
    * 
    * @param transformToPack the transform in which this frame's transform to the
    *           {@code desiredFrame} is stored. Modified.
    * @param desiredFrame the goal frame.
    */
   public void getTransformToDesiredFrame(RigidBodyTransform transformToPack, ReferenceFrame desiredFrame)
   {
      try
      {
         verifySameRoots(desiredFrame);
         
         efficientComputeTransform();
         desiredFrame.efficientComputeTransform();
         
         if (desiredFrame.inverseTransformToRoot != null)
         {
            if (transformToRoot != null)
            {
               transformToPack.set(desiredFrame.inverseTransformToRoot);
               transformToPack.multiply(transformToRoot);
            }
            else
            {
               transformToPack.set(desiredFrame.inverseTransformToRoot);
            }
         }
         else
         {
            if (transformToRoot != null)
            {
               transformToPack.set(transformToRoot);
            }
            else
            {
               transformToPack.setIdentity();
            }
         }
      }
      catch (NotARotationMatrixException e)
      {
         throw new NotARotationMatrixException("Caught exception, this frame: " + frameName + ", other frame: " + desiredFrame.getName() + ", exception:/n" + e.getMessage());
      }
   }

   /**
    * Test whether the given frame is the parent of this frame.
    * 
    * @param frame the query.
    * @return {@code true} if the query is the parent of this frame, {@code false} otherwise.
    */
   public boolean isParentFrame(ReferenceFrame frame)
   {
      return frame == parentFrame;
   }

   /**
    * Test whether the given frame is a child of this frame.
    * 
    * @param frame the query.
    * @return {@code true} if the query is a child of this frame, {@code false} otherwise.
    */
   public boolean isChildFrame(ReferenceFrame frame)
   {
      return frame.isParentFrame(this);
   }

   /**
    * Asserts that this frame and {@code referenceFrame} share the same root frame.
    * 
    * @param referenceFrame the query.
    * @throws RuntimeException if this frame and the query do not share the same root frame.
    */
   public void verifySameRoots(ReferenceFrame referenceFrame)
   {
      if (getRootFrame() != referenceFrame.getRootFrame())
      {
         throw new RuntimeException("Frames do not have same roots. this = " + this + ", referenceFrame = " + referenceFrame);
      }
   }

   /**
    * Returns the internal reference to this frame's transform to the root frame.
    * <p>
    * The transform can be used to transform a geometry object defined in this frame to obtain its
    * equivalent expressed in the root frame.
    * </p>
    * 
    * @return the internal reference to the transform from this frame to the root frame.
    */
   public RigidBodyTransform getTransformToRoot()
   {
      efficientComputeTransform();
      return transformToRoot;
   }

   /**
    * Returns the internal reference to this frame's transform from the root frame to this frame.
    * <p>
    * The transform can be used to transform a geometry object defined in the root frame to obtain
    * its equivalent expressed in this frame.
    * </p>
    * 
    * @return the internal reference to the transform from the root frame to this frame.
    */
   public RigidBodyTransform getInverseTransformToRoot()
   {
      efficientComputeTransform();
      return inverseTransformToRoot;
   }

   private void efficientComputeTransform()
   {
      int chainLength = framesStartingWithRootEndingWithThis.length;

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

   /**
    * Overrides the {@link Object#toString()} method to print this reference frame's name.
    * 
    * @return this frame's name.
    */
   @Override
   public String toString()
   {
      return frameName; // + "\nTransform to Parent = " + this.transformToParent;
   }

   /**
    * Checks if the query holds onto this reference frame.
    * <p>
    * This is usually used from verifying that a geometry is expressed in a specific frame.
    * </p>
    * 
    * @param referenceFrameHolder the query holding a reference frame.
    * @throws ReferenceFrameMismatchException if the query holds onto a different frame than this.
    */
   public void checkReferenceFrameMatch(AbstractReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException
   {
      checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   /**
    * Check if this frame and the query are the same.
    * 
    * @param referenceFrame the query.
    * @throws ReferenceFrameMismatchException if the query and this are two different frame.
    */
   public void checkReferenceFrameMatch(ReferenceFrame referenceFrame) throws ReferenceFrameMismatchException
   {
      if (this != referenceFrame)
      {
         String msg = "Argument's frame " + referenceFrame + " does not match " + this;

         throw new ReferenceFrameMismatchException(msg);
      }
   }

   /**
    * Checks if this frame is equal to {@link #worldFrame}.
    * 
    * @throws RuntimeException if this is not {@link #worldFrame}.
    */
   public void checkIsWorldFrame() throws RuntimeException
   {
      if (!isWorldFrame())
      {
         throw new RuntimeException("Frame " + this + " is not world frame.");
      }
   }

   /**
    * Checks if this is a stationary frame, i.e. not moving with respect to the root frame.
    * 
    * @throws RuntimeException if this is not a stationary frame.
    */
   public void checkIsAStationaryFrame() throws RuntimeException
   {
      if (!isAStationaryFrame())
      {
         throw new RuntimeException("Frame " + this + " is not a stationary frame.");
      }
   }

   /**
    * Checks if this is a z-up frame, i.e. its z-axis is aligned with the root frame's z-axis.
    * 
    * @throws RuntimeException if this is not a z-up frame.
    */
   public void checkIsAZUpFrame() throws RuntimeException
   {
      if (!isZupFrame())
      {
         throw new RuntimeException("Frame " + this + " is not a z-up frame.");
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

         if (transformToRootID != 0)
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

   /**
    * Gets the value of this frame's name based hash code.
    * <p>
    * This is a secondary unique hash code representing this reference frame that is computed based
    * on {@link #frameName} and the parent frame name if any.
    * </p>
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same tree of reference frames, such that it can be used to serialize and deserialize frame
    * information.
    * </p>
    * 
    * @return this frame's name based hash code.
    */
   @Override
   public long getNameBasedHashCode()
   {
      return nameBasedHashCode;
   }

   /**
    * Gets the value of this frame's custom hash code.
    * <p>
    * Somewhat of a hack that allows to enforce two frames that are physically the same but with
    * different names to have the same hash code or to enforce a common frame to have a specific
    * hash code that can be known without holding on its actual instance.
    * </p>
    */
   public long getAdditionalNameBasedHashCode()
   {
      return additionalNameBasedHashCode;
   }

   /**
    * Sets this frame's custom hash code's value.
    * <p>
    * Somewhat of a hack that allows to enforce two frames that are physically the same but with
    * different names to have the same hash code or to enforce a common frame to have a specific
    * hash code that can be known without holding on its actual instance.
    * </p>
    * 
    * @param additionalNameBasedHashCode the new value of this frame's custom hash code.
    */
   public void setAdditionalNameBasedHashCode(long additionalNameBasedHashCode)
   {
      this.additionalNameBasedHashCode = additionalNameBasedHashCode;
   }

   //////////////////////////////////////////////////////////////////////
   /////// This needs to be extract to a new type of reference frame ////
   //////////////////////////////////////////////////////////////////////

   private RigidBodyTransform preCorruptionTransform, postCorruptionTransform;

   /**
    * Please avoid using the corruption methods until we know where they should be moved.
    * <p>
    * The big problem is that it makes the pose any reference frame uncertain as from anywhere it
    * can easily be modified. It was initially created to understand unmodeled offsets in the
    * kinematics of Atlas. It will probably still be needed in the future for the same reason, so
    * let's not remove it but avoid using unless it is strictly necessary.
    * </p>
    * 
    * @param preCorruptionTransform the corruption transform to prepend to this frame's transform to
    *           parent.
    */
   @Deprecated
   public void corruptTransformToParentPreMultiply(RigidBodyTransform preCorruptionTransform)
   {
      if (this.preCorruptionTransform == null)
      {
         this.preCorruptionTransform = new RigidBodyTransform();
      }

      this.preCorruptionTransform.set(preCorruptionTransform);
      update();
   }

   /**
    * Please avoid using the corruption methods until we know where they should be moved.
    * <p>
    * The big problem is that it makes the pose any reference frame uncertain as from anywhere it
    * can easily be modified. It was initially created to understand unmodeled offsets in the
    * kinematics of Atlas. It will probably still be needed in the future for the same reason, so
    * let's not remove it but avoid using unless it is strictly necessary.
    * </p>
    * 
    * @param postCorruptionTransform the corruption transform to append to this frame's transform to
    *           parent.
    */
   @Deprecated
   public void corruptTransformToParentPostMultiply(RigidBodyTransform postCorruptionTransform)
   {
      if (this.postCorruptionTransform == null)
      {
         this.postCorruptionTransform = new RigidBodyTransform();
      }

      this.postCorruptionTransform.set(postCorruptionTransform);
      update();
   }

   //////////////////////////////////////////////////////////////////////
   ////////////////// End of things to be extracted /////////////////////
   //////////////////////////////////////////////////////////////////////
}
