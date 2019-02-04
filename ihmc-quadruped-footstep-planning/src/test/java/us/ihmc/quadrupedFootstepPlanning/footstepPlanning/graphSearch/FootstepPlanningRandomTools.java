package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

public class FootstepPlanningRandomTools
{
   public static FootstepNode createRandomFootstepNode(Random random)
   {
      RobotQuadrant robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
      Point2DReadOnly frontLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
      Point2DReadOnly frontRight= EuclidCoreRandomTools.nextPoint2D(random, 1.0);
      Point2DReadOnly hindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
      Point2DReadOnly hindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);

      return new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight);
   }

   public static FootstepNode createRandomFootstepNode(Random random, double length, double width)
   {
      RobotQuadrant robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
      Point2DReadOnly center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
      double yaw = RandomNumbers.nextDouble(random, -Math.PI * 2.0, Math.PI * 2.0);

      AxisAngle rotation = new AxisAngle(yaw, 0.0, 0.0);

      Vector2D offsetFromCenter = new Vector2D(length, width);
      offsetFromCenter.scale(0.5);

      rotation.transform(offsetFromCenter);

      Point2D frontLeft = new Point2D(center);
      Point2D frontRight = new Point2D(center);
      Point2D hindLeft = new Point2D(center);
      Point2D hindRight = new Point2D(center);
      frontLeft.add(offsetFromCenter);
      frontRight.add(offsetFromCenter.getX(), -offsetFromCenter.getY());
      hindLeft.add(-offsetFromCenter.getX(), offsetFromCenter.getY());
      hindRight.add(-offsetFromCenter.getX(), -offsetFromCenter.getY());

      return new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight);
   }
}
