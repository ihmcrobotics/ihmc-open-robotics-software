package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.scene.DetectableSceneObject;

/**
 *
 */
public class PullDoorPanelWithLeverHandle extends DetectableSceneObject
{
   public PullDoorPanelWithLeverHandle(String name, long uuid)
   {
      super(name, uuid);
   }

   public PullDoorPanelWithLeverHandle(String name, long uuid, ReferenceFrame parentFrame)
   {
      super(name, uuid, parentFrame);
   }
}
