package us.ihmc.sensorProcessing.frames;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.robotModels.FullRobotModel;

public class CommonReferenceFrames implements ReferenceFrames
{
   private final TLongObjectHashMap<ReferenceFrame> hashCodeToReferenceFrameMap = new TLongObjectHashMap<ReferenceFrame>();
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final FullRobotModel fullRobotModel;

   public CommonReferenceFrames(FullRobotModel fullRobotModel)
   {
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      this.fullRobotModel = fullRobotModel;
      hashCodeToReferenceFrameMap.put(CommonReferenceFrameIds.CENTER_OF_MASS_FRAME.getHashId(), centerOfMassFrame);
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
}