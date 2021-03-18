package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

public class PawStepPlanningRandomTools
{
   public static PawNode createRandomFootstepNode(Random random)
   {
      return createRandomFootstepNode(random, 1.0);
   }

   public static PawNode createRandomFootstepNode(Random random, double minMaxXY)
   {
      return new PawNode(RobotQuadrant.generateRandomRobotQuadrant(random),
                         new QuadrantDependentList<>(EuclidCoreRandomTools.nextPoint2D(random, minMaxXY), EuclidCoreRandomTools.nextPoint2D(random, minMaxXY),
                                                          EuclidCoreRandomTools.nextPoint2D(random, minMaxXY), EuclidCoreRandomTools.nextPoint2D(random, minMaxXY)),
                         RandomNumbers.nextDouble(random, Math.PI), 1.0, 0.5);
   }

   public static PawNode createRandomFootstepNode(Random random, double length, double width)
   {
      RobotQuadrant robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
      Point2DReadOnly center = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

      Vector2D offsetFromCenter = new Vector2D(length, width);
      offsetFromCenter.scale(0.5);

      Point2D frontLeft = new Point2D(center);
      Point2D frontRight = new Point2D(center);
      Point2D hindLeft = new Point2D(center);
      Point2D hindRight = new Point2D(center);
      frontLeft.add(offsetFromCenter);
      frontRight.add(offsetFromCenter.getX(), -offsetFromCenter.getY());
      hindLeft.add(-offsetFromCenter.getX(), offsetFromCenter.getY());
      hindRight.add(-offsetFromCenter.getX(), -offsetFromCenter.getY());

      double yaw = PawNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                             hindRight.getX(), hindRight.getY());

      return new PawNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, length, width);
   }
}
