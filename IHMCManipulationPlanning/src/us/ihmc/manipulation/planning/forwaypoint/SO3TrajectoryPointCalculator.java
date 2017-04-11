package us.ihmc.manipulation.planning.forwaypoint;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SO3TrajectoryPointCalculator
{
   private final RecyclingArrayList<FrameSO3TrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(20, FrameSO3TrajectoryPoint.class);
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   
   public SO3TrajectoryPointCalculator()
   {
      
   }
   
   public void appendTrajectoryPoint(Quaternion orientation)
   {
      FrameSO3TrajectoryPoint aTrajectoryQuaternion = trajectoryPoints.add();
      aTrajectoryQuaternion.setToZero(referenceFrame);
      aTrajectoryQuaternion.setTimeToNaN();
      aTrajectoryQuaternion.setOrientation(orientation);
      aTrajectoryQuaternion.setAngularVelocityToNaN();
   }
   
   public void appendTrajectoryPoint(Quaternion orientation, Vector3D angularVelocity)
   {    
      FrameSO3TrajectoryPoint aTrajectoryQuaternion = trajectoryPoints.add();
      aTrajectoryQuaternion.setToZero(referenceFrame);
      aTrajectoryQuaternion.setTimeToNaN();
      aTrajectoryQuaternion.setOrientation(orientation);
      aTrajectoryQuaternion.setAngularVelocity(angularVelocity);      
   }
   
   public void setTrajectoryPointAngularVelocity(int index, Vector3D angularVelocity)
   {
      trajectoryPoints.get(index).setAngularVelocity(angularVelocity);
   }
   
   public void setTrajectoryPointOrientation(int index, Quaternion orientation)
   {
      trajectoryPoints.get(index).setOrientation(orientation);
   }
   
   public void setTrajectoryPoint(int index, Quaternion orientation, Vector3D angularVelocity)
   {
      setTrajectoryPointOrientation(index, orientation);
      setTrajectoryPointAngularVelocity(index, angularVelocity);
   }
   
   // public void ChangeFrame should be added after making FrameEulideanTrajectoryQuaternion extending GeometricObject.
   
   public RecyclingArrayList<FrameSO3TrajectoryPoint> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }
   
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }
   
   public double getTime(int index)
   {
      return getTrajectoryPoints().get(index).getTime();
   }
   
   public void setTrajectoryPointTimes(RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints)
   {
      for(int i =0;i<trajectoryPoints.size();i++)
      {
         double time = trajectoryPoints.get(i).getTime();
         this.trajectoryPoints.get(i).setTime(time);
      }
   }
   
   // This is temporary equation for obtaining angular velocity. /17.03.17/ 
   // w = log(qi-1 ^(-1) * qi)
   // The B-Spline proposed by KimKimShin should be added.
   public void computeTrajectoryPoints()
   {  
      Vector3D angularVelocity = new Vector3D();
      
      for(int i =0;i<getNumberOfTrajectoryPoints()-1;i++)
      {         
         Quaternion quaternionOne = new Quaternion();
         trajectoryPoints.get(i).getOrientation(quaternionOne);
         Quaternion quaternionOneInv = new Quaternion(quaternionOne);
         Quaternion quaternionTwo = new Quaternion();
         trajectoryPoints.get(i+1).getOrientation(quaternionTwo);
         Quaternion quaternionToGo = new Quaternion();
         
         quaternionOneInv.inverse();
         quaternionToGo.multiply(quaternionOneInv, quaternionTwo);
         
         double alpha = Math.acos(quaternionToGo.getS());
         
         angularVelocity.setToZero();   
         angularVelocity.setX(quaternionToGo.getX() / Math.sin(alpha) * alpha);
         angularVelocity.setY(quaternionToGo.getY() / Math.sin(alpha) * alpha);
         angularVelocity.setZ(quaternionToGo.getZ() / Math.sin(alpha) * alpha);
         
         if(Double.isNaN(angularVelocity.getX()) == true || Double.isNaN(angularVelocity.getY()) == true || Double.isNaN(angularVelocity.getZ()) == true)
         {
            angularVelocity.setToZero();
         }
      
         PrintTools.info("aVel ["+i+"] "+angularVelocity.getX()+" "+angularVelocity.getY()+" "+angularVelocity.getZ());
         trajectoryPoints.get(i).setAngularVelocity(angularVelocity);
      }
            
      angularVelocity.setToZero();
      trajectoryPoints.get(getNumberOfTrajectoryPoints()-1).setAngularVelocity(angularVelocity);
   }
}
