package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class ConstrainedRigidBodyData
{
   /**
    * The first way point time.
    */
   private double t0;
   
   /**
    * The final way point time.
    */
   private double tf;
   
   private long rigidBodyNameBasedHashCode;
   
   private SelectionMatrix6D trajectorySelectionMatrix;
   private SelectionMatrix6D explorationSelectionMatrix;
   
   private double explorationUpperLimit;
   private double explorationLowerLimit;
   
   // TODO
   public ConstrainedRigidBodyData(WaypointBasedTrajectoryCommand trajectoryMessage, RigidBodyExplorationConfigurationCommand explorationMessage)
   {
      t0 = trajectoryMessage.getWaypointTime(0);
      tf = trajectoryMessage.getLastWaypointTime();
   }
   
   public ConstrainedRigidBodyData(RigidBodyExplorationConfigurationCommand explorationMessage)
   {
      
   }
   
   public ConstrainedRigidBodyData(long rigidBodyHashCode)
   {
      
   }
   
   
   
   public long getRigidBodyHashCode()
   {
      return rigidBodyNameBasedHashCode;
   }
   
   // TODO
   // this is for the kinematicsSolver.
   public SelectionMatrix6D getSelectionMatrix()
   {
      return new SelectionMatrix6D();
   }
   
   // TODO
   // from trajectoryFunction
   public Pose3D getPoseFromTrajectory(double time)
   {
      double putTime;
      
      if(time < t0)
         putTime = t0;
      else if(time > tf)
         putTime = tf;
      else
         ;
      
      return new Pose3D();
   }

   // TODO
   // default value
   public RigidBodyTransform getControlFrameTransformation()
   {
      return new RigidBodyTransform();
   }
   
   // TODO
   // based on upper lower limit
   // some rigidbodydata has default region
   public RigidBodyTransform getRandomTransformation()
   {
      return new RigidBodyTransform();
   }
}
