package us.ihmc.sensorProcessing.frames;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CommonReferenceFrames implements ReferenceFrames
{
   private final TLongObjectHashMap<ReferenceFrame> nameBasedHashCodeToReferenceFrameMap = new TLongObjectHashMap<ReferenceFrame>();
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final FullRobotModel fullRobotModel;

   public CommonReferenceFrames(FullRobotModel fullRobotModel)
   {
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      this.fullRobotModel = fullRobotModel;
      nameBasedHashCodeToReferenceFrameMap.put(CommonReferenceFrameIds.CENTER_OF_MASS_FRAME.getHashId(), centerOfMassFrame);
   }

   @Override
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public void updateFrames()
   {
      fullRobotModel.updateFrames();
      centerOfMassFrame.update();
   }

   /**
    * This function should map reference frame to custom hash ids, Each reference frame generates a hash id based on its name and its parent name,
    * In some instances it is desirable to also assign a custom id to make reference frames robot agnostic. 
    */
   @Override
   public TLongObjectHashMap<ReferenceFrame> getReferenceFrameDefaultHashIds()
   {
      return null;
   }
}