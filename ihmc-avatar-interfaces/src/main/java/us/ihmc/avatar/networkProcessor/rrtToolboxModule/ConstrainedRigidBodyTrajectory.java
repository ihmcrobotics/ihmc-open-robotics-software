package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class ConstrainedRigidBodyTrajectory
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
   public ConstrainedRigidBodyTrajectory(WaypointBasedTrajectoryCommand trajectoryMessage, RigidBodyExplorationConfigurationCommand explorationMessage)
   {
      t0 = trajectoryMessage.getWaypointTime(0);
      tf = trajectoryMessage.getLastWaypointTime();
   }
   
   public ConstrainedRigidBodyTrajectory(RigidBodyExplorationConfigurationCommand explorationMessage)
   {
      
   }
   
   public ConstrainedRigidBodyTrajectory(long rigidBodyHashCode)
   {
      
   }
   
   
   
   public long getRigidBodyHashCode()
   {
      return rigidBodyNameBasedHashCode;
   }
   
   // this is for the kinematicsSolver.
   public SelectionMatrix6D getSelectionMatrix()
   {      
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      
      if(trajectorySelectionMatrix.getLinearPart().isXSelected() || explorationSelectionMatrix.getLinearPart().isXSelected())
         selectionMatrix.getLinearPart().selectXAxis(true);
      
      if(trajectorySelectionMatrix.getLinearPart().isYSelected() || explorationSelectionMatrix.getLinearPart().isYSelected())
         selectionMatrix.getLinearPart().selectYAxis(true);
      
      if(trajectorySelectionMatrix.getLinearPart().isZSelected() || explorationSelectionMatrix.getLinearPart().isZSelected())
         selectionMatrix.getLinearPart().selectZAxis(true);
      
      if(trajectorySelectionMatrix.getAngularPart().isXSelected() || explorationSelectionMatrix.getAngularPart().isXSelected())
         selectionMatrix.getAngularPart().selectXAxis(true);
      
      if(trajectorySelectionMatrix.getAngularPart().isYSelected() || explorationSelectionMatrix.getAngularPart().isYSelected())
         selectionMatrix.getAngularPart().selectYAxis(true);
      
      if(trajectorySelectionMatrix.getAngularPart().isZSelected() || explorationSelectionMatrix.getAngularPart().isZSelected())
         selectionMatrix.getAngularPart().selectZAxis(true);
      
      return selectionMatrix;
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
