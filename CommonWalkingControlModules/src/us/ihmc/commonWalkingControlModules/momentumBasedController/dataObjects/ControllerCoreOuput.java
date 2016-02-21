package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControllerCoreOuput
{
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;

   public ControllerCoreOuput(CenterOfPressureDataHolder centerOfPressureDataHolder)
   {
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
   }

   public void setDesiredCenterOfPressure(FramePoint2d cop, RigidBody rigidBody)
   {
      centerOfPressureDataHolder.setCenterOfPressure(cop, rigidBody);
   }

   public void getDesiredCenterOfPressure(FramePoint2d copToPack, RigidBody rigidBody)
   {
      centerOfPressureDataHolder.getCenterOfPressure(copToPack, rigidBody);
   }
}
