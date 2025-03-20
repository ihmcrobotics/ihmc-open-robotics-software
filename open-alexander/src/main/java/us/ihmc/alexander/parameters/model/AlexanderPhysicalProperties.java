package us.ihmc.alexander.parameters.model;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface AlexanderPhysicalProperties
{
   double getActualFootLength();

   double getActualFootWidth();

   double getFootWidthForControl();

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
}