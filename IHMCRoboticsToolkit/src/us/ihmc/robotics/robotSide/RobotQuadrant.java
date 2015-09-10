package us.ihmc.robotics.robotSide;

import java.awt.Color;
import java.util.ArrayList;

public enum RobotQuadrant
{
   FRONT_LEFT(RobotEnd.FRONT, RobotSide.LEFT),
   FRONT_RIGHT(RobotEnd.FRONT, RobotSide.RIGHT),
   HIND_RIGHT(RobotEnd.HIND, RobotSide.RIGHT),
   HIND_LEFT(RobotEnd.HIND, RobotSide.LEFT);
   
   public static final RobotQuadrant[] values = values();
   
   private final RobotEnd end;
   private final RobotSide side;

   public static final int FRONT_LEFT_ORDINAL = FRONT_LEFT.ordinal();
   public static final int FRONT_RIGHT_ORDINAL = FRONT_RIGHT.ordinal();
   public static final int HIND_RIGHT_ORDINAL = HIND_RIGHT.ordinal();
   public static final int HIND_LEFT_ORDINAL = HIND_LEFT.ordinal();

   private static final Color[] colorArray = new Color[] { Color.YELLOW, Color.WHITE, Color.BLUE, Color.BLACK };

   private static final Color[] colorArrayForWhiteBackground = new Color[] { Color.YELLOW, Color.PINK, Color.BLUE, Color.BLACK };
   
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
   
   public static final RobotQuadrant getQuadrant(RobotEnd robotEnd, RobotSide robotSide)
   {
      if (robotEnd == RobotEnd.HIND)
      {
         if (robotSide == RobotSide.LEFT)
         {
            return HIND_LEFT;
         }
         else
         {
            return HIND_RIGHT;
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
   
   // Methods:
   public final RobotQuadrant getDiagonalOppositeQuadrant()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return RobotQuadrant.HIND_RIGHT;
         }
   
         case FRONT_RIGHT:
         {
            return RobotQuadrant.HIND_LEFT;
         }
   
         case HIND_RIGHT:
         {
            return RobotQuadrant.FRONT_LEFT;
         }
   
         case HIND_LEFT:
         {
            return RobotQuadrant.FRONT_RIGHT;
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }

   public Color getColor()
   {
      return colorArray[this.ordinal()];
   }

   public Color getColorForWhiteBackground()
   {
      return colorArrayForWhiteBackground[this.ordinal()];
   }

   public final RobotQuadrant getAcrossBodyQuadrant()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return RobotQuadrant.FRONT_RIGHT;
         }
   
         case FRONT_RIGHT:
         {
            return RobotQuadrant.FRONT_LEFT;
         }
   
         case HIND_RIGHT:
         {
            return RobotQuadrant.HIND_LEFT;
         }
   
         case HIND_LEFT:
         {
            return RobotQuadrant.HIND_RIGHT;
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }

   public final String getShortName()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return "FL";
         }
   
         case FRONT_RIGHT:
         {
            return "FR";
         }
   
         case HIND_RIGHT:
         {
            return "HR";
         }
   
         case HIND_LEFT:
         {
            return "HL";
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }

   public static ArrayList<RobotQuadrant> getAllQuadrants()
   {
      ArrayList<RobotQuadrant> quadrantNames = new ArrayList<RobotQuadrant>();
      for (RobotQuadrant quadrantName : RobotQuadrant.values())
      {
         quadrantNames.add(quadrantName);
      }

      return quadrantNames;
   }

   public static ArrayList<RobotQuadrant> getFrontQuadrants()
   {
      ArrayList<RobotQuadrant> quadrantNames = new ArrayList<RobotQuadrant>();
      for (RobotQuadrant quadrantName : RobotQuadrant.values())
      {
         if (quadrantName.isQuadrantInFront())
            quadrantNames.add(quadrantName);
      }

      return quadrantNames;
   }

   public static ArrayList<RobotQuadrant> getHindQuadrants()
   {
      ArrayList<RobotQuadrant> quadrantNames = new ArrayList<RobotQuadrant>();
      for (RobotQuadrant quadrantName : RobotQuadrant.values())
      {
         if (quadrantName.isQuadrantInHind())
            quadrantNames.add(quadrantName);
      }

      return quadrantNames;
   }

   public static ArrayList<RobotQuadrant> getDiagonalOppositeQuadrants(boolean useFrontLeft)
   {
      ArrayList<RobotQuadrant> quadrantNames = new ArrayList<RobotQuadrant>();

      if (useFrontLeft)
      {
         quadrantNames.add(RobotQuadrant.FRONT_LEFT);
         quadrantNames.add(RobotQuadrant.HIND_RIGHT);
      }
      else
      {
         quadrantNames.add(RobotQuadrant.FRONT_RIGHT);
         quadrantNames.add(RobotQuadrant.HIND_LEFT);
      }

      return quadrantNames;
   }

   public final RobotQuadrant getSameSideQuadrant()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return RobotQuadrant.HIND_LEFT;
         }
   
         case FRONT_RIGHT:
         {
            return RobotQuadrant.HIND_RIGHT;
         }
   
         case HIND_RIGHT:
         {
            return RobotQuadrant.FRONT_RIGHT;
         }
   
         case HIND_LEFT:
         {
            return RobotQuadrant.FRONT_LEFT;
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }

   public final boolean isQuadrantOnLeftSide()
   {
      return ((this == FRONT_LEFT) || (this == HIND_LEFT));
   }

   public final boolean isQuadrantOnRightSide()
   {
      return ((this == FRONT_RIGHT) || (this == HIND_RIGHT));
   }

   public final boolean isQuadrantInFront()
   {
      return ((this == FRONT_LEFT) || (this == FRONT_RIGHT));
   }

   public final boolean isQuadrantInHind()
   {
      return ((this == HIND_LEFT) || (this == HIND_RIGHT));
   }

   public final RobotQuadrant getNextRegularGaitSwingQuadrant()
   {
      return getNextRegularGaitSwingQuadrant(this);
   }

   public static final RobotQuadrant getNextRegularGaitSwingQuadrant(RobotQuadrant currentSwingQuadrant)
   {
      switch (currentSwingQuadrant)
      {
         case FRONT_LEFT:
         {
            return RobotQuadrant.HIND_RIGHT;
         }
   
         case FRONT_RIGHT:
         {
            return RobotQuadrant.HIND_LEFT;
         }
   
         case HIND_RIGHT:
         {
            return RobotQuadrant.FRONT_RIGHT;
         }
   
         case HIND_LEFT:
         {
            return RobotQuadrant.FRONT_LEFT;
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }

   public static RobotQuadrant getQuadrantNameFromOrdinal(int quadrantIndex)
   {
      if (quadrantIndex == RobotQuadrant.FRONT_LEFT.ordinal())
         return RobotQuadrant.FRONT_LEFT;
      if (quadrantIndex == RobotQuadrant.FRONT_RIGHT.ordinal())
         return RobotQuadrant.FRONT_RIGHT;
      if (quadrantIndex == RobotQuadrant.HIND_LEFT.ordinal())
         return RobotQuadrant.HIND_LEFT;
      if (quadrantIndex == RobotQuadrant.HIND_RIGHT.ordinal())
         return RobotQuadrant.HIND_RIGHT;

      throw new RuntimeException("Invalid quadrant index: " + quadrantIndex);
   }

   public static RobotQuadrant getQuadrantName(String quadrantName)
   {
      if (quadrantName.equals(RobotQuadrant.FRONT_LEFT.toString()))
         return RobotQuadrant.FRONT_LEFT;
      if (quadrantName.equals(RobotQuadrant.FRONT_RIGHT.toString()))
         return RobotQuadrant.FRONT_RIGHT;
      if (quadrantName.equals(RobotQuadrant.HIND_LEFT.toString()))
         return RobotQuadrant.HIND_LEFT;
      if (quadrantName.equals(RobotQuadrant.HIND_RIGHT.toString()))
         return RobotQuadrant.HIND_RIGHT;

      throw new RuntimeException("Invalid quadrant name: " + quadrantName);
   }
   
   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return "FrontLeft";
         }
   
         case FRONT_RIGHT:
         {
            return "FrontRight";
         }
   
         case HIND_RIGHT:
         {
            return "HindRight";
         }
   
         case HIND_LEFT:
         {
            return "HindLeft";
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }
   
   public String getCamelCaseNameForMiddleOfExpression()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return "frontLeft";
         }
   
         case FRONT_RIGHT:
         {
            return "frontRight";
         }
   
         case HIND_RIGHT:
         {
            return "hindRight";
         }
   
         case HIND_LEFT:
         {
            return "hindLeft";
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }
}
