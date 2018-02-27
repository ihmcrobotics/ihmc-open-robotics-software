package us.ihmc.robotics.robotSide;

import java.util.EnumSet;

public enum RobotSextant implements RobotSegment<RobotSextant>
{
   FRONT_LEFT(RobotSide.LEFT), FRONT_RIGHT(RobotSide.RIGHT), MIDDLE_LEFT(RobotSide.LEFT), MIDDLE_RIGHT(RobotSide.RIGHT), HIND_LEFT(RobotSide.LEFT), HIND_RIGHT(RobotSide.RIGHT);

   public static final EnumSet<RobotSextant> enumSet = EnumSet.allOf(RobotSextant.class);
   public static RobotSextant[] values = values();
   private final RobotSide robotSide;
   
   RobotSextant(RobotSide robotSide)
   {
      this.robotSide = robotSide;
      
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public EnumSet<RobotSextant> getEnumSet()
   {
      return enumSet;
   }

   @Override
   public RobotSextant[] getValues()
   {
      return values;
   }
}
