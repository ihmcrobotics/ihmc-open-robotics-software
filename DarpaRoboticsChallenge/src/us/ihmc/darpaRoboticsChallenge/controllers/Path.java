package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.userInterface.modifiableObjects.PathActivationLevel;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;

public interface Path
{
   public ArrayList<Footstep> generateFootsteps(HeightMapWithPoints heightMap, SideDependentList<ContactablePlaneBody> contactableFeet);

   public void cancel();

   public void activateGraphics(PathActivationLevel activationLevel);

   public void removeFootStep(Footstep footstep);

}
