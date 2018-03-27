package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionNode;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlannerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Wrapper around the {@code CentroidalMotionPlanner} to enable high level objectives to be translated to 
 * motion planning inputs
 * @author Apoorv S
 *
 */
public class WholeBodyMotionManager
{
   private final ReferenceFrame planningFrame;
   private final CentroidalMotionPlanner motionPlanner;
   private final CentroidalMotionNode motionNode;

   private final FramePoint3D initialPosition;
   private final FrameVector3D initialVelocity;
   private final FrameVector3D initialAccleration;

   private final FramePoint3D finalPosition;
   private final FrameVector3D finalVelocity;
   private final FrameVector3D finalAccleration;

   private JumpStateEnum currentState;
   
   public WholeBodyMotionManager(ReferenceFrame planningFrame, CentroidalMotionPlannerParameters parameters)
   {
      this.motionPlanner = new CentroidalMotionPlanner(parameters);
      this.planningFrame = planningFrame;
      this.motionNode = new CentroidalMotionNode(planningFrame);
      this.initialPosition = new FramePoint3D(planningFrame);
      this.initialVelocity = new FrameVector3D(planningFrame);
      this.initialAccleration = new FrameVector3D(planningFrame);
      this.finalPosition = new FramePoint3D(planningFrame);
      this.finalVelocity = new FrameVector3D(planningFrame);
      this.finalAccleration = new FrameVector3D(planningFrame);
   }

   public void setInitialState(FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D accelerationToSet)
   {
      this.initialPosition.setIncludingFrame(positionToSet);
      this.initialPosition.changeFrame(planningFrame);
      this.initialVelocity.setIncludingFrame(velocityToSet);
      this.initialVelocity.changeFrame(planningFrame);
      this.initialAccleration.setIncludingFrame(accelerationToSet);
      this.initialAccleration.changeFrame(planningFrame);
   }
   
   public void setFinalState(FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D accelerationToSet)
   {
      this.finalPosition.setIncludingFrame(positionToSet);
      this.finalPosition.changeFrame(planningFrame);
      this.finalVelocity.setIncludingFrame(velocityToSet);
      this.finalVelocity.changeFrame(planningFrame);
      this.finalAccleration.setIncludingFrame(accelerationToSet);
      this.finalAccleration.changeFrame(planningFrame);
   }

   public void compute()
   {
      
   }
}
