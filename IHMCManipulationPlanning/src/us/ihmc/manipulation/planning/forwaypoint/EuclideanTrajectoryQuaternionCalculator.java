package us.ihmc.manipulation.planning.forwaypoint;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;

public class EuclideanTrajectoryQuaternionCalculator extends EuclideanTrajectoryPointCalculator
{
   private final ArrayList<FrameEuclideanTrajectoryQuaternion> trajectoryQuaternions = new ArrayList<FrameEuclideanTrajectoryQuaternion>();
   
   public EuclideanTrajectoryQuaternionCalculator()
   {
      
   }
   
   public void appendTrajectoryQuaternion(Quaternion orientation)
   {
      FrameEuclideanTrajectoryQuaternion aTrajectoryQuaternion = new FrameEuclideanTrajectoryQuaternion();
      aTrajectoryQuaternion.setQuaternion(orientation);
      trajectoryQuaternions.add(aTrajectoryQuaternion);
   }
   
   public void appendTrajectoryQuaternion(Quaternion orientation, Vector3D angularVelocity)
   {
      FrameEuclideanTrajectoryQuaternion aTrajectoryQuaternion = new FrameEuclideanTrajectoryQuaternion();
      aTrajectoryQuaternion.setQuaternion(orientation);
      aTrajectoryQuaternion.setAngularVelocity(angularVelocity);
      trajectoryQuaternions.add(aTrajectoryQuaternion);
   }
   
   public void setTrajectoryQuaternion(int index, Vector3D angularVelocity)
   {
      trajectoryQuaternions.get(index).setAngularVelocity(angularVelocity);
   }
   
   public void setTrajectoryQuaternion(int index, Quaternion orientation)
   {
      trajectoryQuaternions.get(index).setQuaternion(orientation);
   }
   
   public void setTrajectoryQuaternion(int index, Quaternion orientation, Vector3D angularVelocity)
   {
      setTrajectoryQuaternion(index, orientation);
      setTrajectoryQuaternion(index, angularVelocity);
   }
   
   // public void ChangeFrame should be added after making FrameEulideanTrajectoryQuaternion extending GeometricObject.
   
   public ArrayList<FrameEuclideanTrajectoryQuaternion> getTrajectoryQuaternions()
   {
      return trajectoryQuaternions;
   }
   
   public int getNumberOfTrajectoryQuaternions()
   {
      return trajectoryQuaternions.size();
   }
   
   public double getTime(int index)
   {
      return getTrajectoryPoints().get(index).getTime();
   }
   
   // This is temporary equation for obtaining angular velocity. /17.03.17/ 
   // w = log(qi-1 ^(-1) * qi)
   // The B-Spline proposed by KimKimShin should be added.
   public void computeTrajectoryQuaternions()
   {  
      Vector3D angularVelocity = new Vector3D();
      
      for(int i =0;i<getNumberOfTrajectoryQuaternions()-1;i++)
      {         
         Quaternion quaternionOne = new Quaternion(trajectoryQuaternions.get(i).getQuaternion());
         Quaternion quaternionOneInv = new Quaternion(quaternionOne);
         Quaternion quaternionTwo = new Quaternion(trajectoryQuaternions.get(i+1).getQuaternion());
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
         trajectoryQuaternions.get(i).setAngularVelocity(angularVelocity);
      }
            
      angularVelocity.setToZero();
      trajectoryQuaternions.get(getNumberOfTrajectoryQuaternions()-1).setAngularVelocity(angularVelocity);
   }
}
