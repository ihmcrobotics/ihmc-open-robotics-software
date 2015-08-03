package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.robotics.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public interface PathGeneratorManager
{
   public FullRobotModel getFullRobotModel();
   public ReferenceFrames getReferenceFrames();
   public SideDependentList<ContactablePlaneBody> getBipedFeet();
}
