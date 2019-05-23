package us.ihmc.robotics.robotSide;

import java.awt.Color;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Random;

import org.apache.commons.lang3.ArrayUtils;

public enum RobotQuadrant implements RobotSegment<RobotQuadrant>
{
   FRONT_LEFT(RobotEnd.FRONT, RobotSide.LEFT),
   FRONT_RIGHT(RobotEnd.FRONT, RobotSide.RIGHT),
   HIND_RIGHT(RobotEnd.HIND, RobotSide.RIGHT),
   HIND_LEFT(RobotEnd.HIND, RobotSide.LEFT);
   
   public static final EnumSet<RobotQuadrant> enumSet = EnumSet.allOf(RobotQuadrant.class);
   public static final RobotQuadrant[] values = values();
   public static final RobotQuadrant[] reversedValues = values();
   static 
   {
      ArrayUtils.reverse(reversedValues);
   }
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

public static RobotQuadrant generateRandomRobotQuadrant(Random random)
   {
      if (random.nextBoolean())
      {
         if (random.nextBoolean())
            return FRONT_LEFT;
         else
            return FRONT_RIGHT;
      }
      else
      {
         if (random.nextBoolean())
            return HIND_LEFT;
         else
            return HIND_RIGHT;
      }
   }

   public RobotEnd getEnd()
   {
      return end;
   }
   
   public RobotEnd getOppositeEnd()
   {
      return end.getOppositeEnd();
   }
  
   public RobotSide getSide()
   {
      return side;
   }
   
   public RobotSide getOppositeSide()
   {
      return side.getOppositeSide();
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
   
   public final RobotQuadrant getAcrossBodyFrontQuadrant()
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
   
   public final RobotQuadrant getAcrossBodyHindQuadrant()
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
   
   public final RobotQuadrant getNextClockwiseQuadrant()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return RobotQuadrant.FRONT_RIGHT;
         }
   
         case FRONT_RIGHT:
         {
            return RobotQuadrant.HIND_RIGHT;
         }
   
         case HIND_RIGHT:
         {
            return RobotQuadrant.HIND_LEFT;
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
   
   public final RobotQuadrant getNextCounterClockwiseQuadrant()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return RobotQuadrant.HIND_LEFT;
         }
   
         case HIND_LEFT:
         {
            return RobotQuadrant.HIND_RIGHT;
         }
   
         case HIND_RIGHT:
         {
            return RobotQuadrant.FRONT_RIGHT;
         }
   
         case FRONT_RIGHT:
         {
            return RobotQuadrant.FRONT_LEFT;
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
   
   public final RobotQuadrant getNextReversedRegularGaitSwingQuadrant()
   {
      return getNextReversedRegularGaitSwingQuadrant(this);
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
   
   public static final RobotQuadrant getNextReversedRegularGaitSwingQuadrant(RobotQuadrant currentSwingQuadrant)
   {
      switch (currentSwingQuadrant)
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

   public static RobotQuadrant getQuadrantNameFromOrdinal(int quadrantIndex)
   {
      return values[quadrantIndex];
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

   public static RobotQuadrant guessQuadrantFromName(String name)
   {
      if(name.contains("front") || name.contains("Front"))
      {
         if(name.contains("left") || name.contains("Left"))
            return RobotQuadrant.FRONT_LEFT;
         else if(name.contains("right") || name.contains("Right"))
            return RobotQuadrant.FRONT_RIGHT;
      }
      else if(name.contains("hind") || name.contains("Hind"))
      {
         if(name.contains("left") || name.contains("Left"))
            return RobotQuadrant.HIND_LEFT;
         else if(name.contains("right") || name.contains("Right"))
            return RobotQuadrant.HIND_RIGHT;
      }
      else if (name.contains("hip") || name.contains("Hip") || name.contains("HIP") || name.contains("knee") || name.contains("Knee") || name.contains("KNEE"))
      {
         if(name.contains("left") || name.contains("Left") || name.contains("LEFT"))
            return RobotQuadrant.HIND_LEFT;
         else if(name.contains("right") || name.contains("Right") || name.contains("RIGHT"))
            return RobotQuadrant.HIND_RIGHT;
      }
      else if (name.contains("shoulder") || name.contains("Shoulder") || name.contains("SHOULDER") || name.contains("elbow") || name.contains("Elbow") || name.contains("ELBOW"))
      {
         if(name.contains("left") || name.contains("Left") || name.contains("LEFT"))
            return RobotQuadrant.FRONT_LEFT;
         else if(name.contains("right") || name.contains("Right") || name.contains("RIGHT"))
            return RobotQuadrant.FRONT_RIGHT;
      }

      return null;
   }

   public String getPascalCaseName()
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

   public String getTitleCaseName()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return "Front Left";
         }
         case FRONT_RIGHT:
         {
            return "Front Right";
         }
         case HIND_RIGHT:
         {
            return "Hind Right";
         }
         case HIND_LEFT:
         {
            return "Hind Left";
         }

         default:
         {
            throw new RuntimeException();
         }
      }
   }
   
   public String getCamelCaseName()
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
   
   public String getUnderBarName()
   {
      switch (this)
      {
         case FRONT_LEFT:
         {
            return "front_left";
         }
   
         case FRONT_RIGHT:
         {
            return "front_right";
         }
   
         case HIND_RIGHT:
         {
            return "hind_right";
         }
   
         case HIND_LEFT:
         {
            return "hind_left";
         }
   
         default:
         {
            throw new RuntimeException();
         }
      }
   }
   
   /**
    * @deprecated Use getPascalCaseName() instead.
    */
   public String getCamelCaseNameForMiddleOfExpression()
   {
      return getPascalCaseName();
   }
   
   /**
    * @deprecated Use getCamelCaseName() instead.
    */
   public String getCamelCaseNameForStartOfExpression()
   {
      return getCamelCaseName();
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static RobotQuadrant fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }

   @Override
   public EnumSet<RobotQuadrant> getEnumSet()
   {
      return enumSet;
   }

   @Override
   public RobotQuadrant[] getValues()
   {
      return values;
   }

   @Override
   public Class<RobotQuadrant> getClassType()
   {
      return RobotQuadrant.class;
   }
}
