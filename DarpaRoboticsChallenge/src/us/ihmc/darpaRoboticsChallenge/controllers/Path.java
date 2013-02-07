package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.userInterface.modifiableObjects.PathActivationLevel;
import us.ihmc.utilities.math.dataStructures.HeightMap;

public interface Path
{
   public ArrayList<Footstep> generateFootsteps(HeightMap heightMap);

   public void cancel();

   public void activateGraphics(PathActivationLevel activationLevel);

}
