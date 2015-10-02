package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

import java.util.Collection;

import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface FootstepGenerator
{
   Collection<? extends Footstep> generateDesiredFootstepList();    // Should be only required function

   void setPoseFinderParams(double footstepFittingBufferSize, double boundingBoxForFootstepHeightFindingSideLength);

   void setHeightMap(HeightMapWithPoints heightMap, SideDependentList<ContactablePlaneBody> contactableFeet);
   
   boolean hasDisplacement();
}
