package us.ihmc.communication.crdt;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class CRDTDetachableReferenceFrame
{
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalRigidBodyTransform transformToParent;
   private final DetachableReferenceFrame detachableReferenceFrame;

   public CRDTDetachableReferenceFrame(ReferenceFrameLibrary referenceFrameLibrary,
                                       CRDTUnidirectionalString parentFrameName,
                                       CRDTUnidirectionalRigidBodyTransform transformToParent)
   {
      this.parentFrameName = parentFrameName;
      this.transformToParent = transformToParent;

      detachableReferenceFrame = new DetachableReferenceFrame(referenceFrameLibrary, transformToParent.getValueReadOnly());
   }

   public void update()
   {
      detachableReferenceFrame.update(parentFrameName.getValue());
   }

   public void changeFrame(String newParentFrameName)
   {
      parentFrameName.setValue(newParentFrameName);
      detachableReferenceFrame.changeFrame(newParentFrameName, transformToParent.accessValue());
   }

   public boolean isChildOfWorld()
   {
      return detachableReferenceFrame.isChildOfWorld();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return detachableReferenceFrame.getReferenceFrame();
   }
}
