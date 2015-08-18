package us.ihmc.robotics.robotSide;

import java.awt.Color;

import sun.applet.AppletIllegalArgumentException;

public class RobotQuadrant
{
   public static final RobotQuadrant BACK_LEFT = new RobotQuadrant(RobotEnd.BACK, RobotSide.LEFT);
   public static final RobotQuadrant BACK_RIGHT = new RobotQuadrant(RobotEnd.BACK, RobotSide.RIGHT);
   public static final RobotQuadrant FRONT_LEFT = new RobotQuadrant(RobotEnd.FRONT, RobotSide.LEFT);
   public static final RobotQuadrant FRONT_RIGHT = new RobotQuadrant(RobotEnd.FRONT, RobotSide.RIGHT);
   
   public static final RobotQuadrant[] values = new RobotQuadrant[] {BACK_LEFT, BACK_RIGHT, FRONT_LEFT, FRONT_RIGHT};
   
   private final RobotEnd end;
   private final RobotSide side;
   
   public static RobotQuadrant getQuadrant(RobotEnd robotEnd, RobotSide robotSide)
   {
      if (robotEnd == RobotEnd.BACK)
      {
         if (robotSide == RobotSide.LEFT)
         {
            return BACK_LEFT;
         }
         else
         {
            return BACK_RIGHT;
         }
      }
      else
      {
         if (robotSide == RobotSide.LEFT)
         {
            return FRONT_LEFT;
         }
         else
         {
            return FRONT_RIGHT;
         }
      }
   }
   
   public static RobotQuadrant getDiagonalOppositeQuadrant(RobotQuadrant robotQuadrant)
   {
      if (robotQuadrant.getEnd() == RobotEnd.BACK)
      {
         if (robotQuadrant.getSide() == RobotSide.LEFT)
         {
            return FRONT_RIGHT;
         }
         else
         {
            return FRONT_LEFT;
         }
      }
      else
      {
         if (robotQuadrant.getSide() == RobotSide.LEFT)
         {
            return BACK_RIGHT;
         }
         else
         {
            return BACK_LEFT;
         }
      }
   }
   
   public static RobotQuadrant getAcrossBodyQuadrant(RobotQuadrant robotQuadrant)
   {
      if (robotQuadrant.getEnd() == RobotEnd.BACK)
      {
         if (robotQuadrant.getSide() == RobotSide.LEFT)
         {
            return BACK_RIGHT;
         }
         else
         {
            return BACK_LEFT;
         }
      }
      else
      {
         if (robotQuadrant.getSide() == RobotSide.LEFT)
         {
            return FRONT_RIGHT;
         }
         else
         {
            return FRONT_LEFT;
         }
      }
   }
   
   public static final RobotQuadrant getNextRegularGaitSwingQuadrant(RobotQuadrant robotQuadrant)
   {
      if (robotQuadrant.getEnd() == RobotEnd.BACK)
      {
         if (robotQuadrant.getSide() == RobotSide.LEFT)
         {
            return FRONT_LEFT;
         }
         else
         {
            return FRONT_RIGHT;
         }
      }
      else
      {
         if (robotQuadrant.getSide() == RobotSide.LEFT)
         {
            return BACK_RIGHT;
         }
         else
         {
            return BACK_LEFT;
         }
      }
   }
   
   private RobotQuadrant(RobotEnd end, RobotSide side)
   {
      this.end = end;
      this.side = side;
   }
   
   public RobotEnd getEnd()
   {
      return end;
   }

   public RobotSide getSide()
   {
      return side;
   }
   
   public Color getColor()
   {
      if (end == RobotEnd.BACK)
      {
         if (side == RobotSide.LEFT)
         {
            return Color.BLUE;
         }
         else
         {
            return Color.BLACK;
         }
      }
      else
      {
         if (side == RobotSide.LEFT)
         {
            return Color.WHITE;
         }
         else
         {
            return Color.YELLOW;
         }
      }
   }
   
   public Color getColorForWhiteBackground()
   {
      if (end == RobotEnd.BACK)
      {
         if (side == RobotSide.LEFT)
         {
            return Color.BLACK;
         }
         else
         {
            return Color.BLUE;
         }
      }
      else
      {
         if (side == RobotSide.LEFT)
         {
            return Color.YELLOW;
         }
         else
         {
            return Color.PINK;
         }
      }
   }
   
   public int getBDIQuadrantIndex()
   {
      if (end == RobotEnd.BACK)
      {
         if (side == RobotSide.LEFT)
         {
            return 2;
         }
         else
         {
            return 3;
         }
      }
      else
      {
         if (side == RobotSide.LEFT)
         {
            return 0;
         }
         else
         {
            return 1;
         }
      }
   }
}
