package us.ihmc.manipulation.planning.trajectory;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.transformables.Pose;

public class EndEffectorLinearTrajectory implements EndEffectorTrajectory
{
   private ArrayList<LinearTrajectory> trajectories = new ArrayList<LinearTrajectory>();
   private double trajectoryTime = 0;
   private Pose initialPose;
   
   public EndEffectorLinearTrajectory()
   {
      
   }
   
   public void clearTrajectories()
   {
      trajectories.clear();
   }
   
   public void setInitialPose(Pose pose)
   {
      initialPose = pose;
   }
   
   public void addLinearTrajectory(Pose pose, double trajectoryTime)
   {
      if(trajectories.size() == 0)
      {
         LinearTrajectory appendTrajectory = new LinearTrajectory(initialPose, pose, trajectoryTime);
         addLinearTrajectory(appendTrajectory);
      }
      else
      {
         LinearTrajectory lastTrajectory = trajectories.get(trajectories.size()-1);
         Pose localInitialPose = lastTrajectory.getPose(lastTrajectory.getTrajectoryTime());
         LinearTrajectory appendTrajectory = new LinearTrajectory(localInitialPose, pose, trajectoryTime);
         addLinearTrajectory(appendTrajectory);
      }
   }
   
   public void addLinearTrajectory(LinearTrajectory appendTrajectory)
   {
      trajectories.add(appendTrajectory);
   }
   
   @Override
   public double getTrajectoryTime()
   {
      trajectoryTime = 0;
      
      for(int i=0;i<trajectories.size();i++)
      {
         trajectoryTime = trajectoryTime + trajectories.get(i).getTrajectoryTime();
      }
      
      return trajectoryTime;
   }

   @Override
   public Pose getEndEffectorPose(double time)
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
}
