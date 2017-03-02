package us.ihmc.robotics.robotSide;

public enum RobotSextant
{
   FRONT_LEFT(RobotSide.LEFT), FRONT_RIGHT(RobotSide.RIGHT), MIDDLE_LEFT(RobotSide.LEFT), MIDDLE_RIGHT(RobotSide.RIGHT), HIND_LEFT(RobotSide.LEFT), HIND_RIGHT(RobotSide.RIGHT);
   
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
   
}
