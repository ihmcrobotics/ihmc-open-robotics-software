package us.ihmc.communication.crdt;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

import java.util.function.Function;

public class CRDTDetachableReferenceFrame
{
   private final CRDTBidirectionalString parentFrameName;
   private final CRDTBidirectionalRigidBodyTransform transformToParent;
   private final DetachableReferenceFrame detachableReferenceFrame;

   public CRDTDetachableReferenceFrame(Function<String, ReferenceFrame> frameFunction,
                                       CRDTBidirectionalString parentFrameName,
                                       CRDTBidirectionalRigidBodyTransform transformToParent)
   {
      this.parentFrameName = parentFrameName;
      this.transformToParent = transformToParent;

      detachableReferenceFrame = new DetachableReferenceFrame(frameFunction, transformToParent.getValueReadOnly());
   }

   public void update()
   {
      detachableReferenceFrame.update(parentFrameName.getValue());
   }

   public void changeFrame(String newParentFrameName)
   {
      parentFrameName.setValue(newParentFrameName);
      detachableReferenceFrame.changeFrame(newParentFrameName, transformToParent.getValueAndModify());
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
