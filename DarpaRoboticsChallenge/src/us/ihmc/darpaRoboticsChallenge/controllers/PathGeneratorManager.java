package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.robotSide.SideDependentList;

public interface PathGeneratorManager
{
   public FullRobotModel getFullRobotModel();
   public ReferenceFrames getReferenceFrames();
   public SideDependentList<ContactablePlaneBody> getBipedFeet();
}
