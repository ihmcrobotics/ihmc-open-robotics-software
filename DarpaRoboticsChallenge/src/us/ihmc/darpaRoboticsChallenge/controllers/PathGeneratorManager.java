package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

public interface PathGeneratorManager
{
   public FullRobotModel getFullRobotModel();
   public HumanoidReferenceFrames getReferenceFrames();
   public SideDependentList<ContactablePlaneBody> getBipedFeet();
}
