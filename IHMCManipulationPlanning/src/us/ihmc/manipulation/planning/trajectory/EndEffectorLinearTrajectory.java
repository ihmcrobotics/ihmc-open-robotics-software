package us.ihmc.manipulation.planning.trajectory;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class EndEffectorLinearTrajectory extends EndEffectorTrajectory
{
   private ArrayList<LinearTrajectory> trajectories = new ArrayList<LinearTrajectory>();   
   private Pose3D initialPose;   
   
   public EndEffectorLinearTrajectory()
   {
      
   }
       
   public void setInitialPose(Pose3D pose)
   {
      trajectories.clear();
      initialPose = pose;
      trajectoryTime = 0;
   }
   
   public void addLinearTrajectory(Pose3D pose, double trajectoryTime)
   {
      if(trajectories.size() == 0)
      {
         LinearTrajectory appendTrajectory = new LinearTrajectory(initialPose, pose, trajectoryTime);
         addLinearTrajectory(appendTrajectory);
      }
      else
      {
         LinearTrajectory lastTrajectory = trajectories.get(trajectories.size()-1);
         Pose3D localInitialPose = lastTrajectory.getPose(lastTrajectory.getTrajectoryTime());
         LinearTrajectory appendTrajectory = new LinearTrajectory(localInitialPose, pose, trajectoryTime);
         addLinearTrajectory(appendTrajectory);
      }
      this.trajectoryTime = this.trajectoryTime + trajectoryTime;
   }
   
   public void addLinearTrajectory(LinearTrajectory appendTrajectory)
   {
      trajectories.add(appendTrajectory);
      this.trajectoryTime = this.trajectoryTime + appendTrajectory.getTrajectoryTime();
   }
   


   @Override
   public Pose3D getEndEffectorPose(double time)
   {
      double integratedTime = 0;
      double localTime = 0;
      
      for(int i=0;i<trajectories.size();i++)
      {
         if((time - integratedTime) < trajectories.get(i).getTrajectoryTime())
         {
            localTime = time - integratedTime;
            return trajectories.get(i).getPose(localTime);
         }
         else
         {
            integratedTime = integratedTime + trajectories.get(i).getTrajectoryTime();
         }
      }
      
      PrintTools.warn("requested time is so far. return final pose of the last linear path");
      
      LinearTrajectory lastTrajectory = trajectories.get(trajectories.size()-1);
      
      return lastTrajectory.getPose(lastTrajectory.getTrajectoryTime());
   }

   /*
    * 
    */
   @Override
   public HandTrajectoryMessage getEndEffectorTrajectoryMessage(ReferenceFrame midFeetFrame)
   {
      return new HandTrajectoryMessage();
   }
   

}
