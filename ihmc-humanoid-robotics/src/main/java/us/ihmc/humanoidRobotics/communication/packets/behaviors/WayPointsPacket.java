package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class WayPointsPacket extends Packet<WayPointsPacket>
{
   public RobotSide robotSide;
   
   public int numberOfWayPoints;
   
   public Point3D[] positionOfWayPoints;
   public Quaternion[] orientationOfWayPoints;
   
   public double trajectoryTime;
   
   // VRUI mode should be able to select RobotSide.
   // 
   
   public WayPointsPacket()
   {
      
   }
   
   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }
   
   public void setWayPoints(Pose3D[] poseOfWayPoints, double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
      
      this.numberOfWayPoints = poseOfWayPoints.length;
      
      this.positionOfWayPoints = new Point3D[this.numberOfWayPoints];
      this.orientationOfWayPoints = new Quaternion[this.numberOfWayPoints];
      
      for(int i=0;i<this.numberOfWayPoints;i++)
      {
         this.positionOfWayPoints[i] = new Point3D(poseOfWayPoints[i].getPosition());
         this.orientationOfWayPoints[i] = new Quaternion(poseOfWayPoints[i].getOrientation());
      }
   }
   
   @Override
   public boolean epsilonEquals(WayPointsPacket other, double epsilon)
   {
      if(this.numberOfWayPoints != other.numberOfWayPoints)
         return false;
      
      if(this.positionOfWayPoints.length != other.positionOfWayPoints.length)
         return false;
      if(this.orientationOfWayPoints.length != other.orientationOfWayPoints.length)
         return false;
      
      for(int i=0;i<this.positionOfWayPoints.length;i++)
      {
         if(!this.positionOfWayPoints[i].epsilonEquals(other.positionOfWayPoints[i], epsilon))
            return false;
      }
      
      for(int i=0;i<this.orientationOfWayPoints.length;i++)
      {
         if(!this.orientationOfWayPoints[i].epsilonEquals(other.orientationOfWayPoints[i], epsilon))
            return false;
      }
      
      if(!MathTools.epsilonEquals(trajectoryTime, other.trajectoryTime, epsilon))
         return false;
      
      if(!this.robotSide.equals(other.robotSide))
         return false;
            
      return true;
   }

}
