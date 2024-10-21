package us.ihmc.avatar.drcRobot;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface RobotPhysicalProperties
{
   SideDependentList<RigidBodyTransform> getHandAttachmentPlateToWristTransforms();

   SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms();


   double getThighLength();

   double getShinLength();


   double getActualFootLength();

   double getActualFootWidth();


   double getFootForwardForControl();

   double getFootBackForControl();

   double getFootLengthForControl();

   double getToeWidthForControl();

   double getFootWidthForControl();
}
