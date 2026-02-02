package us.ihmc.zulu.parameters.model;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface ZuluPhysicalProperties
{
   double getActualFootLength();

   double getActualFootWidth();

   double getActualFootBack();

   double getFootWidthForControl();

   double getAnkleHeight();

   default double getToeWidthForControl()
   {
      return getFootWidthForControl();
   }

   double getFootLengthForControl();

   double getFootBackForControl();

   double getFootForwardForControl();

   double getThighLength();

   double getShinLength();

   SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms();

   RigidBodyTransform getHandControlFrameToWristTransform(RobotSide side);
}