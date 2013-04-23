package us.ihmc.darpaRoboticsChallenge.initialSetup;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;

public class SquaredUpDRCDemo01OutsidePen extends SquaredUpDRCRobotInitialSetup
{
   private final double offsetX, offsetY;
   private final double yaw;
   private Vector3d newOffset = null;
   
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
      
      if (newOffset == null)
      {
         newOffset = new Vector3d(offset);
         newOffset.add(new Vector3d(offsetX, offsetY, 0.0));
      }
      
      offset.set(newOffset);
      
      robot.setPositionInWorld(offset);
      robot.setOrientation(yaw, 0.0, 0.0);
   }

}
