package us.ihmc.robotics.robotSide;

import java.awt.Color;

import us.ihmc.tools.FormattingTools;

public enum RobotQuadrant
{
   BACK_LEFT(RobotEnd.BACK, RobotSide.LEFT),
   BACK_RIGHT(RobotEnd.BACK, RobotSide.RIGHT),
   FRONT_LEFT(RobotEnd.FRONT, RobotSide.LEFT),
   FRONT_RIGHT(RobotEnd.FRONT, RobotSide.RIGHT);
   
   public static final RobotQuadrant[] values = values();
   
   private final RobotEnd end;
   private final RobotSide side;
   
   public static final RobotQuadrant getQuadrant(RobotEnd robotEnd, RobotSide robotSide)
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
   
   public static final RobotQuadrant getDiagonalOppositeQuadrant(RobotQuadrant robotQuadrant)
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
   
   public static final RobotQuadrant getAcrossBodyQuadrant(RobotQuadrant robotQuadrant)
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
   
   public String getCamelCaseNameForStartOfExpression()
   {
      return FormattingTools.lowerCaseFirstLetter(getCamelCaseNameForMiddleOfExpression());
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      if (side == RobotSide.RIGHT)
      {
         if (end == RobotEnd.BACK)
         {
            return "backRight";
         }
         else
         {
            return "frontRight";
         }
      }
      else
      {
         if (end == RobotEnd.BACK)
         {
            return "backLeft";
         }
         else
         {
            return "frontLeft";
         }
      }
   }
}
