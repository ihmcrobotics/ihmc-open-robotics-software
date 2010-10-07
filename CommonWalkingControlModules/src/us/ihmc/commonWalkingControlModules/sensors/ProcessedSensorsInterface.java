package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.commonWalkingControlModules.RobotSide;

public interface ProcessedSensorsInterface
{

     public abstract double getTime();
     
     public abstract double getKneeAngle(RobotSide robotSide);
   
}
