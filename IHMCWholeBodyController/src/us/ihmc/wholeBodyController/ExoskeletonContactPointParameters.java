package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public abstract class ExoskeletonContactPointParameters
{
   public abstract List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap();

   public abstract SideDependentList<ArrayList<Point2d>> getFootContactPoints();
   
   public abstract ContactableBodiesFactory getContactableBodiesFactory();

   public abstract void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel);
}
