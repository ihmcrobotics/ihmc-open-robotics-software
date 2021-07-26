package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class ReferenceFrameMissingTools
{
   /**
    * Creates a reference frame with the transform to it's parent maintained by the user.
    * <p>
    * The {@code transformToParent} should describe the pose of the new frame expressed in its parent
    * frame.
    * </p>
    * <p>
    *    Note: It <strong>is</strong> necessary to call update on this reference frame.
    * </p>
    *
    *
    * @param frameName         the name of the new frame.
    * @param parentFrame       the parent frame of the new reference frame.
    * @param transformToParent the transform that can be used to transform a geometry object the new
    *                          frame to its parent frame. Not modified.
    * @return the new reference frame.
    */
   public static ReferenceFrame constructFrameWithChangingTransformToParent(String frameName,
                                                                            ReferenceFrame parentFrame,
                                                                            RigidBodyTransformReadOnly transformToParent)
   {
      boolean isZupFrame = parentFrame.isZupFrame() && transformToParent.isRotation2D();
      boolean isAStationaryFrame = parentFrame.isAStationaryFrame();

      return new ReferenceFrame(frameName, parentFrame, transformToParent, isAStationaryFrame, isZupFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParentToUpdate)
         {
            transformToParentToUpdate.set(transformToParent);
         }
      };
   }
}
