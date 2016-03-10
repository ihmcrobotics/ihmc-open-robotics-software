package us.ihmc.quadrupedRobotics.trot;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum TrotPair
{
   TROT_RIGHT(RobotQuadrant.FRONT_RIGHT, RobotQuadrant.HIND_LEFT), TROT_LEFT(RobotQuadrant.FRONT_LEFT, RobotQuadrant.HIND_RIGHT);

   public static final TrotPair[] values = values();
   private final RobotQuadrant[] quadrants = new RobotQuadrant[2];

   TrotPair(RobotQuadrant frontQuadrant, RobotQuadrant hindQuadrant)
   {
      quadrants[0] = frontQuadrant;
      quadrants[1] = hindQuadrant;
   }

   public RobotQuadrant[] quadrants()
   {
      return quadrants;
   }

   public RobotQuadrant getFrontQuadrant()
   {
      return quadrants[0];
   }
   
   public RobotQuadrant getHindQuadrant()
   {
      return quadrants[1];
   }
}
