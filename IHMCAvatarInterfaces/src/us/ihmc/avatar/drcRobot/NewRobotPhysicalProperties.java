package us.ihmc.avatar.drcRobot;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface NewRobotPhysicalProperties extends DRCRobotPhysicalProperties
{

   double getMassScalePower();

   double getModelScale();

   SideDependentList<RigidBodyTransform> getHandAttachmentPlateToWristTransforms();

   SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms();

   double getThighLength();

   double getShinLength();

   double getFootForward();

   double getFootStartToetaperFromBack();

   double getActualFootLength();

   double getActualFootWidth();

   double getFootBackForControl();

   double getFootLengthForControl();

   double getToeWidthForControl();

   double getFootWidthForControl();

   double getPelvisToFoot();

   double getAnkleHeight();

}
