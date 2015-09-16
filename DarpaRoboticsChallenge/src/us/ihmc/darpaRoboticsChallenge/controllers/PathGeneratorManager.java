package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface PathGeneratorManager
{
   public FullRobotModel getFullRobotModel();
   public HumanoidReferenceFrames getReferenceFrames();
   public SideDependentList<ContactablePlaneBody> getBipedFeet();
}
