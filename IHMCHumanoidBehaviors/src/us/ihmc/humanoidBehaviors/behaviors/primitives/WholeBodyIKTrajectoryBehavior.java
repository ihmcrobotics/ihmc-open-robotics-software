package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.math.trajectories.TrajectoryND;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.LockLevel;
import us.ihmc.wholeBodyController.WholeBodyTrajectory;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WholeBodyIKTrajectoryBehavior extends BehaviorInterface
{
   private final BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet" + behaviorName, registry);
   private final BooleanYoVariable hasComputationBeenDone = new BooleanYoVariable("hasComputationBeenDone" + behaviorName, registry);
   private final BooleanYoVariable sendPacket = new BooleanYoVariable("solutionHaveBeenFound" + behaviorName, registry);
   
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final WholeBodyTrajectory wholeBodyTrajectory;
   private final SDFFullRobotModel actualFullRobotModel;
   private final SDFFullRobotModel desiredFullRobotModel;
   private final DoubleYoVariable yoTime;
   
   private WholeBodyTrajectoryPacket wholeBodyTrajectoryPacket;
   private double trajectoryTime;
   private double startTime;

   public WholeBodyIKTrajectoryBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge,
         WholeBodyControllerParameters wholeBodyControllerParameters, SDFFullRobotModel actualFullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      wholeBodyIKSolver = wholeBodyControllerParameters.createWholeBodyIkSolver();
      
      wholeBodyIKSolver.getConfiguration().setMaxNumberOfAutomaticReseeds(6);
      double positionErrorTolerance = 0.02;
      double orientationErrorTolerance = 0.2;
      wholeBodyIKSolver.taskEndEffectorPosition.get(RobotSide.RIGHT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      wholeBodyIKSolver.taskEndEffectorPosition.get(RobotSide.LEFT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      wholeBodyIKSolver.taskEndEffectorRotation.get(RobotSide.RIGHT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      wholeBodyIKSolver.taskEndEffectorRotation.get(RobotSide.LEFT).setErrorTolerance(positionErrorTolerance, orientationErrorTolerance);
      
      double maxJointVelocity = 1.5;
      double maxJointAcceleration = 15.0;
      double maxDistanceInTaskSpaceBetweenWaypoints = 0.1;
      wholeBodyTrajectory = new WholeBodyTrajectory(actualFullRobotModel, maxJointVelocity, maxJointAcceleration, maxDistanceInTaskSpaceBetweenWaypoints);
      
      this.actualFullRobotModel = actualFullRobotModel;
      this.desiredFullRobotModel = wholeBodyControllerParameters.createFullRobotModel();
      this.yoTime = yoTime;
   }
   
   public void setLockLevel(LockLevel lockLevel)
   {
      wholeBodyIKSolver.getConfiguration().setLockLevel(lockLevel);
   }
   
   public void setMaxReseeds(int maxReseeds)
   {
      wholeBodyIKSolver.getConfiguration().setMaxNumberOfAutomaticReseeds(maxReseeds);
   }
   
   /**
    * Use this function to set the input and use the behavior to calculate a IK trajectory
    * 
    * @param controlledDoFLeft
    * @param palmTargetLeft set null if no left target
    * @param controlledDoFRight
    * @param palmTargetRight set null if no right target
    */
   public void setInput(ControlledDoF controlledDoFLeft, FramePose palmTargetLeft, ControlledDoF controlledDoFRight, FramePose palmTargetRight)
   {
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.LEFT, controlledDoFLeft);
      if (palmTargetLeft != null)
      {
         wholeBodyIKSolver.setGripperPalmTarget( RobotSide.LEFT, palmTargetLeft);
         hasInputBeenSet.set(true);
      }
      
      wholeBodyIKSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.RIGHT, controlledDoFRight);
      if (palmTargetRight != null)
      {
         wholeBodyIKSolver.setGripperPalmTarget( RobotSide.RIGHT, palmTargetRight);
         hasInputBeenSet.set(true);
      }
   }
   
   /**
    * Use this function to set the input to a pre-calculated trajectory packet and send it to the controller
    * 
    * @param wholeBodyTrajectoryPacket
    */
   public void setInput(WholeBodyTrajectoryPacket wholeBodyTrajectoryPacket)
   {
      this.wholeBodyTrajectoryPacket = wholeBodyTrajectoryPacket;
      hasInputBeenSet.set(true);
      hasComputationBeenDone.set(true);
      sendPacket.set(true);
   }

   @Override
   public void doControl()
   {
      if (isPaused())
      {
         return;
      }
      
      if (hasInputBeenSet.getBooleanValue() && !hasComputationBeenDone.getBooleanValue())
      {
         try
         {
            ComputeResult result = wholeBodyIKSolver.compute(actualFullRobotModel, desiredFullRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);
            
            TrajectoryND trajectory = wholeBodyTrajectory.createTaskSpaceTrajectory(wholeBodyIKSolver, actualFullRobotModel, desiredFullRobotModel);
            wholeBodyTrajectoryPacket = wholeBodyTrajectory.convertTrajectoryToPacket(trajectory);
            
            hasComputationBeenDone.set(true);
            if (result == ComputeResult.SUCCEEDED)
            {
               sendPacket.set(true);
            }
            else
            {
               sendPacket.set(false);
               System.out.println("No solution found!");
            }
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
      
      if (sendPacket.getBooleanValue())
      {
         wholeBodyTrajectoryPacket.setDestination(PacketDestination.CONTROLLER);
         sendPacketToController(wholeBodyTrajectoryPacket);
         
         wholeBodyTrajectoryPacket.setDestination(PacketDestination.UI);
         sendPacketToNetworkProcessor(wholeBodyTrajectoryPacket);
         
         int numWaypoints = wholeBodyTrajectoryPacket.numWaypoints;
         trajectoryTime = wholeBodyTrajectoryPacket.timeAtWaypoint[numWaypoints-1] - wholeBodyTrajectoryPacket.timeAtWaypoint[0];
         startTime = yoTime.getDoubleValue();
         sendPacket.set(false);
      }
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      
   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override
   public void enableActions()
   {
      
   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   public boolean isDone()
   {
      boolean trajectoryTimePassed = yoTime.getDoubleValue() - startTime > trajectoryTime;
      return trajectoryTimePassed;
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
      reset();
   }

   @Override
   public void initialize()
   {
      defaultInitialize();
      reset();
   }
   
   private void reset()
   {
      hasInputBeenSet.set(false);
      hasComputationBeenDone.set(false);
      sendPacket.set(false);
      
      startTime = Double.POSITIVE_INFINITY;
      trajectoryTime = Double.POSITIVE_INFINITY;
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }

}
