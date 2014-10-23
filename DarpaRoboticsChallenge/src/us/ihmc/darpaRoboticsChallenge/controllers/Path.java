package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.userInterface.modifiableObjects.PathActivationLevel;
import us.ihmc.utilities.math.dataStructures.HeightMap;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

public interface Path
{
   public ArrayList<Footstep> generateFootsteps(HeightMap heightMap, SideDependentList<ContactablePlaneBody> contactableFeet);

   public void cancel();

   public void activateGraphics(PathActivationLevel activationLevel);

   public void removeFootStep(Footstep footstep);

}
