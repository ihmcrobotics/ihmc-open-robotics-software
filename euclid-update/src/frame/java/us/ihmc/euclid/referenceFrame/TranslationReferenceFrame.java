package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * This defines a mutable {@link ReferenceFrame}, where the translation to the parent {@link ReferenceFrame}, which is accessible as {@link #getParent()} can be
 * updated via {@link #setTranslationAndUpdate(Tuple3DReadOnly)} or {@link #setTranslationAndUpdate(FrameTuple3DReadOnly)}.
 */
public class TranslationReferenceFrame extends ReferenceFrame
{
   /**
    * The translation of this frame relative to its parent.
    */
   public final FrameVector3D translationVector;

   /**
    * <p>
    * Creates a new translation reference frame as a child of the given {@param parentFrame} and initializes the transform to its parent frame.
    * </p>
    * <p>
    * This new reference frame is defined in the {@param parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the parent frame can be modified at runtime by changing the transform in the methods
    * {@link #setTranslationAndUpdate(FrameTuple3DReadOnly)} and {@link #setTranslationAndUpdate(Tuple3DReadOnly)}.
    * </p>
    *
    * @param frameName   the name of the new frame.
    * @param parentFrame the parent of the frame. It has to extend a {@link ReferenceFrame}.
    */
   public TranslationReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, false, parentFrame.isZupFrame());

      translationVector = new FrameVector3D(parentFrame);
   }

   /**
    * <p>
    * Creates a new translation reference frame as a child of the given {@param parentFrame} and initializes the transform to its parent frame.
    * </p>
    * <p>
    * This new reference frame is defined with a constant translation of {@param translationToParent} relative to {@param parentFrame} and moves with it.
    * </p>
    * <p>
    * Its pose with respect to the parent frame can be modified at runtime by changing the transform in the methods
    * {@link #setTranslationAndUpdate(FrameTuple3DReadOnly)} and {@link #setTranslationAndUpdate(Tuple3DReadOnly)}.
    * </p>
    *
    * @param frameName           the name of the new frame.
    * @param parentFrame         the parent of the frame. It has to extend a {@link ReferenceFrame}.
    * @param translationToParent the initial transform to the parent frame.
    */
   public TranslationReferenceFrame(String frameName, ReferenceFrame parentFrame, FrameVector3DReadOnly translationToParent)
   {
      this(frameName, parentFrame);
      setTranslationAndUpdate(translationToParent);
   }

   /**
    * {@inheritDoc}
    **/
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParentToPack)
   {
      transformToParentToPack.setIdentity();
      transformToParentToPack.getTranslation().set(translationVector);
   }

   /**
    * Sets the translation of this reference frame relative to its parent. The frame of the passed in tuple {@param translation} must be defined as
    * {@link #getParent()}. If it is not, this method will throw a {@link ReferenceFrameMismatchException}. To avoid performing a frame check, please use
    * {@link #setTranslationAndUpdate(Tuple3DReadOnly)}
    *
    * @param translation translate relative to parent to set.
    */
   public void setTranslationAndUpdate(FrameTuple3DReadOnly translation)
   {
      translationVector.checkReferenceFrameMatch(translation);
      setTranslationAndUpdate((Tuple3DReadOnly) translation);
   }

   /**
    * Sets the translation of this reference frame relative to its parent. The frame of the passed in tuple {@param translation} is assumed to be the same as
    * {@link #getParent()}. This method does not perform a frame check. For frame safe operations, see {@link #setTranslationAndUpdate(FrameTuple3DReadOnly)}.
    *
    * @param translation translate relative to parent to set.
    */
   public void setTranslationAndUpdate(Tuple3DReadOnly translation)
   {
      translationVector.set(translation);
      this.update();
   }
}  
