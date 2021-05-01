package us.ihmc.quadrupedRobotics.model;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public interface QuadrupedPhysicalProperties
{
   Vector3D getOffsetFromJointBeforeFootToSole(RobotQuadrant robotQuadrant);
   ArrayList<Point2D> getFootGroundContactPoints(RobotQuadrant robotQuadrant);
   QuadrantDependentList<ArrayList<Point2D>> getFeetGroundContactPoints();
   double getNominalBodyHeight();
   boolean trustFootSwitchesInSwing();
   boolean trustFootSwitchesInSupport();
}
