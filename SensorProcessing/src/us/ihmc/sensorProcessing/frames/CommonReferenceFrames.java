package us.ihmc.sensorProcessing.frames;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CommonReferenceFrames implements ReferenceFrames
{
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final SDFFullRobotModel fullRobotModel;

   public CommonReferenceFrames(SDFFullRobotModel fullRobotModel)
   {
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      this.fullRobotModel = fullRobotModel;
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