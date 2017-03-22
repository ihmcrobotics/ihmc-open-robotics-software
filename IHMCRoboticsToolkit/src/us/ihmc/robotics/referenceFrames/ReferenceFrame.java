package us.ihmc.robotics.referenceFrames;

import java.io.Serializable;
import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeHolder;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;

/**
 * <p>ReferenceFrame </p>
 *
 * <p>Description: ReferenceFrame is used to represent a reference coordinate frame.  One constructor allows the creation
 * of a "root" frame with no parent.  The other creates a child frame that has a reference and transform
 * to a parent.  HumanoidReferenceFrames are used in classes like FramePoint to indicate which frame the point is defined in.</p>
 *
 */
public abstract class ReferenceFrame implements Serializable, NameBasedHashCodeHolder
{
   private static final long serialVersionUID = 9129810880579453658L;
   protected final String frameName;
   private final long nameBasedHashCode;
   protected final ReferenceFrame parentFrame;
   private final ReferenceFrame[] framesStartingWithRootEndingWithThis;

   private final RigidBodyTransform transformToParent;

   // These need to be longs instead of integers or they'll role over too soon. With longs, you get at least 100 years of runtime.
   protected static long nextTransformToRootID = 1;
   private long transformToRootID = Long.MIN_VALUE;
   private final RigidBodyTransform transformToRoot;
   private final RigidBodyTransform inverseTransformToRoot;

   private final boolean isBodyCenteredFrame;
   private final boolean isWorldFrame;
   private final boolean isZupFrame;

   private static final ReferenceFrame worldFrame = constructAWorldFrame("World");

   public static ReferenceFrame constructAWorldFrame(String frameName)
   {
      ReferenceFrame ret = new ReferenceFrame(frameName, false, true, true)
      {
         private static final long serialVersionUID = -8828178814213025690L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      return ret;
   }

   public static ReferenceFrame constructARootFrame(String frameName)
   {
      return constructARootFrame(frameName, false, false, false);
   }

   public static ReferenceFrame constructARootFrame(String frameName, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame)
   {
      ReferenceFrame ret = new ReferenceFrame(frameName, isBodyCenteredFrame, isWorldFrame, isZupFrame)
      {
         private static final long serialVersionUID = -6427490298776551499L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      return ret;
   }

   public static ReferenceFrame constructReferenceFrameFromPointAndZAxis(String frameName, FramePoint point, FrameVector zAxis)
   {
      point.checkReferenceFrameMatch(zAxis.getReferenceFrame());

      RigidBodyTransform transformToParent = createTransformFromPointAndZAxis(point, zAxis);

      return constructFrameWithUnchangingTransformToParent(frameName, point.getReferenceFrame(), transformToParent);
   }
   
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
      Vector3D translation = new Vector3D();
      point.get(translation);
      transformToDesired.setTranslation(translation);

      return constructFrameWithUnchangingTransformToParent(frameName, point.getReferenceFrame(), transformToDesired);
   }

   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      RigidBodyTransform transformFromParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      return constructFrameWithUnchangingTransformFromParent(frameName, parentFrame, transformFromParent);
   }

   public ReferenceFrame copyAndAimAxisAtPoint(Axis axisToAlign, FramePoint targetToAimAt)
   {
      ReferenceFrame initialFrame = targetToAimAt.getReferenceFrame();
      
      targetToAimAt.changeFrame(this);
      FrameVector targetRelativeToCurrentFrame = new FrameVector(this, targetToAimAt.getX(), targetToAimAt.getY(), targetToAimAt.getZ());
      targetToAimAt.changeFrame(initialFrame);
      
      return copyAndAlignAxisWithVector(axisToAlign, targetRelativeToCurrentFrame);
   }
   
   public ReferenceFrame copyAndAlignAxisWithVector(Axis axisToAlign, FrameVector alignAxisWithThis)
   {
      FrameVector currentXYZAxis = new FrameVector();

      switch (axisToAlign)
      {
      case X:
         currentXYZAxis.setIncludingFrame(this, 1.0, 0.0, 0.0);
         break;
      case Y:
         currentXYZAxis.setIncludingFrame(this, 0.0, 1.0, 0.0);
         break;
      case Z:
         currentXYZAxis.setIncludingFrame(this, 0.0, 0.0, 1.0);
         break;

      default:
         break;
      }
      
      ReferenceFrame initialFrame = alignAxisWithThis.getReferenceFrame();
      alignAxisWithThis.changeFrame(currentXYZAxis.getReferenceFrame());
      AxisAngle rotationToDesired = new AxisAngle();
      EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(currentXYZAxis.getVector(), alignAxisWithThis.getVector(), rotationToDesired);
      alignAxisWithThis.changeFrame(initialFrame);
      
      RigidBodyTransform transformToDesired = new RigidBodyTransform();
      transformToDesired.setRotationAndZeroTranslation(rotationToDesired);
      
      TransformReferenceFrame ret = new TransformReferenceFrame("desiredHandFrame", this, transformToDesired);
      
      return ret;
   }

   public static ReferenceFrame constructBodyZUpFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      //      if (!RotationFunctions.isRotationProper(transformToParent))
      //         throw new RuntimeException("Rotation not normalized: " + transformToParent);
      ReferenceFrame ret = new ReferenceFrame(frameName, parentFrame, true, false, true)
      {
         private static final long serialVersionUID = 5370847059108953557L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      ret.transformToParent.set(transformToParent);

      return ret;
   }
   
   public static ReferenceFrame constructFrameWithUnchangingTransformFromParent(String frameName, ReferenceFrame parentFrame,
         RigidBodyTransform transformFromParent)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform(transformFromParent);
      transformToParent.invert();
      
      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);
   }

   public static ReferenceFrame constructBodyFrameWithUnchangingTranslationFromParent(String frameName, ReferenceFrame parentFrame,
         Vector3D translationFromParent)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslation(translationFromParent);

      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent, true, false, false);
   }

   public static ReferenceFrame constructBodyFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent, true, false, false);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      return constructFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent, false, false, false);
   }

   public static ReferenceFrame constructFrameWithUnchangingTransformToParent(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent,
         boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame)
   {
      //      if (!RotationFunctions.isRotationProper(transformToParent))
      //         throw new RuntimeException("Rotation not normalized: " + transformToParent);
      ReferenceFrame ret = new ReferenceFrame(frameName, parentFrame, isBodyCenteredFrame, isWorldFrame, isZupFrame)
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

   /** Return the world reference frame that is a root reference frame. If located at (0, 0, 0) and its coordinate system is such as the z-axis is up, x-axis is forward, and y-axis points to the left. */
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

      this.isBodyCenteredFrame = false;
      this.isWorldFrame = false;
      this.isZupFrame = false;
   }

   private static ReferenceFrame[] constructFramesStartingWithRootEndingWithThis(ReferenceFrame thisFrame)
   {
      ReferenceFrame parentFrame = thisFrame.parentFrame;
      if (parentFrame == null)
      {
         return new ReferenceFrame[] { thisFrame };
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
    * This constructor creates a "top level" reference frame with the specified name.
    * The parent frame and transforms are null.
    *
    * @param frameName String
    */
   public ReferenceFrame(String frameName, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame)
   {
      this.frameName = frameName;
      nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(frameName);
      this.parentFrame = null;
      this.transformToRootID = 0;

      this.framesStartingWithRootEndingWithThis = new ReferenceFrame[] { this };

      this.transformToRoot = null;
      this.inverseTransformToRoot = null;
      this.transformToParent = null;

      this.isBodyCenteredFrame = isBodyCenteredFrame;
      this.isWorldFrame = isWorldFrame;
      this.isZupFrame = isZupFrame;
   }

   /**
    * This constructor creates a child reference frame with the specified name.
    * The parent frame can be a "top level" frame or another child.
    * The Transform defines how to convert a vector from the parent frame to the child frame.
    * An inverse transform is created automatically based on the provided frame.
    *
    * @param frameName String
    * @param parentFrame Frame
    * @param transformToParent Transform3D
    */
   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isBodyCenteredFrame, boolean isWorldFrame,
         boolean isZupFrame)
   {
      nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(frameName, parentFrame.getName());
      this.frameName = frameName;
      this.parentFrame = parentFrame;
      this.framesStartingWithRootEndingWithThis = constructFramesStartingWithRootEndingWithThis(this);

      this.transformToRoot = new RigidBodyTransform();
      this.inverseTransformToRoot = new RigidBodyTransform();

      transformToParent.normalizeRotationPart();
      this.transformToParent = transformToParent;

      this.isBodyCenteredFrame = isBodyCenteredFrame;
      this.isWorldFrame = isWorldFrame;
      this.isZupFrame = isZupFrame;
   }

   public ReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame)
   {
      this.frameName = frameName;
      this.parentFrame = parentFrame;
      this.framesStartingWithRootEndingWithThis = constructFramesStartingWithRootEndingWithThis(this);

      this.isBodyCenteredFrame = isBodyCenteredFrame;
      this.isWorldFrame = isWorldFrame;
      this.isZupFrame = isZupFrame;

      this.transformToRoot = new RigidBodyTransform();
      this.inverseTransformToRoot = new RigidBodyTransform();
      this.transformToParent = new RigidBodyTransform();
      
      if(parentFrame != null)
      {
         nameBasedHashCode = NameBasedHashCodeTools.combineHashCodes(frameName, parentFrame.getName());
      }
      else
      {
         nameBasedHashCode = NameBasedHashCodeTools.computeStringHashCode(frameName);
      }
   }

   public boolean isBodyCenteredFrame()
   {
      return isBodyCenteredFrame;
   }

   public boolean isWorldFrame()
   {
      return isWorldFrame;
   }

   public boolean isZupFrame()
   {
      return isZupFrame;
   }

   /**
    * The user must call update each tick.
    * It will then call updateTransformToParent.
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

   // TODO: Make this method private and don't use it anywhere else!
   protected void setTransformToParent(RigidBodyTransform transformToParent)
   {
      this.transformToParent.set(transformToParent);
      this.transformToRootID = Long.MIN_VALUE;
//      transformToParent.normalize();
   }

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
    * Returns a Transform3D that can be applied to a vector defined in this frame
    * in order to obtain the equivalent vector in the parent frame
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

   /**
    * Creates a transform that transforms to the given point and rotates to make the z axis align with the
    * normal vector.
    */
   private static RigidBodyTransform createTransformFromPointAndZAxis(FramePoint point, FrameVector zAxis)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      ret.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(zAxis.getVectorCopy()));
      Vector3D translation = new Vector3D();
      point.get(translation);
      ret.setTranslation(translation);

      return ret;
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

   protected void checkRepInvariants()
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
            throw new RuntimeException(
                  "Root frames don't have transformToParent or transformToRoot defined. This is so RMI still works with frames since Transform3D is not serializable.");
         }

         if (transformToRoot != null)
         {
            throw new RuntimeException(
                  "Root frames don't have transformToParent or transformToRoot defined. This is so RMI still works with frames since Transform3D is not serializable.");
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
}
