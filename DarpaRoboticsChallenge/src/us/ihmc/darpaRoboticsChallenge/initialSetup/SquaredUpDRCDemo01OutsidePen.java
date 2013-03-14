package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;

public class SquaredUpDRCDemo01OutsidePen extends SquaredUpDRCRobotInitialSetup
{
   private final double offsetX, offsetY;
   private final double yaw;
   
   public SquaredUpDRCDemo01OutsidePen(double groundZ)
   {
      super(groundZ);
      
      offsetX = 3.0;
      offsetY = 12.0;
      
      yaw = Math.PI/2.0;
   }

   public void initializeRobot(SDFRobot robot)
   {  
      super.initializeRobot(robot);
      
      offset.setX(offset.getX() + offsetX);
      offset.setY(offset.getY() + offsetY);
      
      robot.setPositionInWorld(offset);
      robot.setOrientation(yaw, 0.0, 0.0);
   }

}
