package us.ihmc.darpaRoboticsChallenge.controllers;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.userInterface.modifiableObjects.GraphicsActivationLevel;

public interface Path
{
   public ArrayList<Footstep> generateFootsteps();

   public void cancel();

   public void activateGraphics(GraphicsActivationLevel activationLevel);

}
