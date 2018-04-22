package us.ihmc.commonWalkingControlModules.controlModules.flight;

import java.util.List;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionNode;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlannerParameters;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.ControlModuleHelper;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.ForceTrajectory;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.PositionTrajectory;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Wrapper around the {@code CentroidalMotionPlanner} to enable high level objectives to be translated to 
 * motion planning inputs
 * @author Apoorv S
 *
 */
public class WholeBodyMotionPlanner
{
   public static final int numberOfForceCoefficients = ControlModuleHelper.forceCoefficients;
   public static final int maxNumberOfSegments = 20;

   private final ReferenceFrame planningFrame = ReferenceFrame.getWorldFrame();
   private final CentroidalMotionPlanner motionPlanner;
   private final CentroidalMotionNode motionNode;

   private final FramePoint3D initialPosition;
   private final FrameVector3D initialLinearVelocity;
   private final FrameVector3D initialGroundReactionForce;
   private final FrameVector3D initialOrientation;
   private final FrameVector3D initialAngularVelocity;
   private final FrameVector3D initialTorque;

   private final FramePoint3D finalPosition;
   private final FrameVector3D finalLinearVelocity;
   private final FrameVector3D finalGroundReactionForce;
   private final FrameVector3D finalOrientation;
   private final FrameVector3D finalAngularVelocity;
   private final FrameVector3D finalTorque;

   private final double gravityZ;
   private final double robotMass;
   private final double plannerDT;

   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final FramePoint3D tempCentroid = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempOrientationVector = new FrameVector3D();
   private final FrameVector3D nominalForce;
   private final FrameVector3D nominalForceRate;
   private final FrameVector3D nominalTorque;
   private final FrameVector3D nominalTorqueRate;

   private final FramePoint3D positionLowerBoundDelta;
   private final FramePoint3D positionUpperBoundDelta;
   private final FramePoint3D positionJumpUpperBoundDelta;
   private final FrameVector3D orientationLowerBoundDelta;
   private final FrameVector3D orientationUpperBoundDelta;

   private final FramePoint3D positionLowerBound;
   private final FramePoint3D positionUpperBound;
   private final FramePoint3D positionJumpUpperBound;
   private final FrameVector3D orientationLowerBound;
   private final FrameVector3D orientationUpperBound;

   private final FrameVector3D positionWeight;
   private final FrameVector3D velocityWeight;
   private final FrameVector3D orientationWeight;
   private final FrameVector3D angularVelocityWeight;

   public WholeBodyMotionPlanner(CentroidalMotionPlannerParameters parameters, YoVariableRegistry registry)
   {
      this.plannerDT = parameters.getDeltaTMin();
      this.gravityZ = parameters.getGravityZ();
      this.robotMass = parameters.getRobotMass();
      this.motionPlanner = new CentroidalMotionPlanner(parameters, registry);
      this.motionNode = new CentroidalMotionNode(planningFrame);
      
      this.initialPosition = new FramePoint3D(planningFrame);
      this.initialLinearVelocity = new FrameVector3D(planningFrame);
      this.initialGroundReactionForce = new FrameVector3D(planningFrame);
      this.initialOrientation = new FrameVector3D(planningFrame);
      this.initialAngularVelocity = new FrameVector3D(planningFrame);
      this.initialTorque = new FrameVector3D(planningFrame);
      
      this.finalPosition = new FramePoint3D(planningFrame);
      this.finalLinearVelocity = new FrameVector3D(planningFrame);
      this.finalGroundReactionForce = new FrameVector3D(planningFrame);
      this.finalOrientation = new FrameVector3D(planningFrame);
      this.finalAngularVelocity = new FrameVector3D(planningFrame);
      this.finalTorque = new FrameVector3D(planningFrame);
      
      this.nominalForce = new FrameVector3D(planningFrame, 0.0, 0.0, -gravityZ * robotMass);
      this.nominalForceRate = new FrameVector3D(planningFrame);
      this.nominalTorque = new FrameVector3D(planningFrame);
      this.nominalTorqueRate = new FrameVector3D(planningFrame);

      this.positionLowerBoundDelta = new FramePoint3D(planningFrame, parameters.getPositionLowerLimits());
      this.positionUpperBoundDelta = new FramePoint3D(planningFrame, parameters.getPositionUpperLimits());
      this.positionJumpUpperBoundDelta = new FramePoint3D(planningFrame, parameters.getJumpPositionUpperLimits());
      this.orientationLowerBoundDelta = new FrameVector3D(planningFrame, parameters.getOrientationLowerLimits());
      this.orientationUpperBoundDelta = new FrameVector3D(planningFrame, parameters.getOrientationUpperLimits());

      this.positionLowerBound = new FramePoint3D(planningFrame);
      this.positionUpperBound = new FramePoint3D(planningFrame);
      this.positionJumpUpperBound = new FramePoint3D(planningFrame);
      this.orientationLowerBound = new FrameVector3D(planningFrame);
      this.orientationUpperBound = new FrameVector3D(planningFrame);

      double defaultPositionWeight = parameters.getDefaultMotionPlanningPositionObjectiveWeight();
      this.positionWeight = new FrameVector3D(planningFrame, defaultPositionWeight, defaultPositionWeight, defaultPositionWeight);
      double defaultLinearVelocityWeight = parameters.getDefaultMotionPlanningVelocityObjectiveWeight();
      this.velocityWeight = new FrameVector3D(planningFrame, defaultLinearVelocityWeight, defaultLinearVelocityWeight, defaultLinearVelocityWeight);

      double defaultOrientationWeight = parameters.getDefaultMotionPlanningOrientationObjectiveWeight();
      this.orientationWeight = new FrameVector3D(planningFrame, defaultOrientationWeight, defaultOrientationWeight, defaultOrientationWeight);
      double defaultAngularVelocityWeight = parameters.getDefaultMotionPlanningAngularVelocityObjectiveWeight();
      this.angularVelocityWeight = new FrameVector3D(planningFrame, defaultAngularVelocityWeight, defaultAngularVelocityWeight, defaultAngularVelocityWeight);
   }

   public void setInitialState(FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D groundReactionForceToSet)
   {
      PrintTools.debug("Got new intial conditions: Position: " + positionToSet.toString() + ", Velocity: " + velocityToSet.toString() + ", GroundReaction: "
            + groundReactionForceToSet.toString());
      this.initialPosition.setIncludingFrame(positionToSet);
      this.initialPosition.changeFrame(planningFrame);
      this.initialLinearVelocity.setIncludingFrame(velocityToSet);
      this.initialLinearVelocity.changeFrame(planningFrame);
      this.initialGroundReactionForce.setIncludingFrame(groundReactionForceToSet);
      this.initialGroundReactionForce.changeFrame(planningFrame);
   }

   public void setFinalState(FramePoint3D positionToSet, FrameVector3D velocityToSet, FrameVector3D groundReactionForce)
   {
      PrintTools.debug("Got new final conditions: Position: " + positionToSet.toString() + ", Velocity: " + velocityToSet.toString() + ", GroundReaction: "
            + groundReactionForce.toString());
      this.finalPosition.setIncludingFrame(positionToSet);
      this.finalPosition.changeFrame(planningFrame);
      this.finalLinearVelocity.setIncludingFrame(velocityToSet);
      this.finalLinearVelocity.changeFrame(planningFrame);
      this.finalGroundReactionForce.setIncludingFrame(groundReactionForce);
      this.finalGroundReactionForce.changeFrame(planningFrame);
   }

   public void processContactStatesAndGenerateMotionNodesForPlanning(List<ContactState> contactStateList)
   {
      if (contactStateList.size() < 1)
         return;
      double nodeTime = 0.0;
      motionNode.reset();
      motionNode.setTime(nodeTime);
      motionNode.setPositionConstraint(initialPosition);
      motionNode.setLinearVelocityConstraint(initialLinearVelocity);
      motionNode.setForceConstraint(initialGroundReactionForce);
      motionNode.setZeroForceRateConstraint();
      motionNode.setOrientationConstraint(initialOrientation);
      motionNode.setAngularVelocityConstraint(initialAngularVelocity);
      motionNode.setTorqueConstraint(initialTorque);
      motionNode.setZeroTorqueRateConstraint();
      motionPlanner.submitNode(motionNode);
      for (int i = 0; i < contactStateList.size() - 1; i++)
      {
         ContactState contactState = contactStateList.get(i);
         ContactType contactType = contactState.getContactType();
         ContactType nextContactType = contactStateList.get(i + 1).getContactType();
         contactState.getSupportPolygon(tempPolygon);
         contactState.getCoMPosition(tempCentroid);
         positionUpperBound.setIncludingFrame(tempCentroid);
         positionUpperBound.add(positionUpperBoundDelta);
         positionLowerBound.setIncludingFrame(tempCentroid);
         positionLowerBound.add(positionLowerBoundDelta);
         
         contactState.getCoMOrientation(tempOrientation);
         tempOrientationVector.setIncludingFrame(tempOrientation.getReferenceFrame(), tempOrientation.getRoll(), tempOrientation.getPitch(),
                                                 tempOrientation.getYaw());
         orientationUpperBound.setIncludingFrame(tempOrientationVector);
         orientationUpperBound.add(orientationUpperBoundDelta);
         orientationLowerBound.setIncludingFrame(tempOrientationVector);
         orientationLowerBound.add(orientationLowerBoundDelta);
         
         double contactStateDuration = contactState.getDuration();
         int numberOfNodesInState = (int) Math.ceil(contactStateDuration / plannerDT);
         for (int j = 0; j < numberOfNodesInState - 1; j++)
         {
            nodeTime += plannerDT;
            motionNode.reset();
            motionNode.setTime(nodeTime);
            switch (contactType)
            {
            case NO_SUPPORT:
               motionNode.setZeroForceConstraint();
               motionNode.setZeroForceRateConstraint();
               motionNode.setZeroTorqueConstraint();
               motionNode.setZeroTorqueRateConstraint();
               break;
            case DOUBLE_SUPPORT:
               motionNode.setForceObjective(nominalForce);
               motionNode.setForceRateObjective(nominalForceRate);
               motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
               motionNode.setTorqueObjective(nominalTorque);
               motionNode.setTorqueRateObjective(nominalTorqueRate);
               motionNode.setOrientationInequalities(orientationUpperBound, orientationLowerBound);
               break;
            default:
               throw new RuntimeException("Unknown support state. Cannot populate motion planner nodes");
            }
            motionPlanner.submitNode(motionNode);
         }
         double finalNodeDuration = contactStateDuration - (numberOfNodesInState - 1) * plannerDT;
         nodeTime += finalNodeDuration;
         motionNode.reset();
         motionNode.setTime(nodeTime);
         if (contactType == ContactType.NO_SUPPORT || nextContactType == ContactType.NO_SUPPORT)
         {
            motionNode.setZeroForceConstraint();
            motionNode.setZeroForceRateConstraint();
            motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
            motionNode.setZeroTorqueConstraint();
            motionNode.setZeroTorqueRateConstraint();
            motionNode.setOrientationInequalities(orientationUpperBound, orientationLowerBound);
         }
         else
         {
            motionNode.setForceObjective(nominalForce);
            motionNode.setForceRateObjective(nominalForceRate);
            motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
            motionNode.setTorqueObjective(nominalTorque);
            motionNode.setTorqueRateObjective(nominalTorqueRate);
            motionNode.setOrientationInequalities(orientationUpperBound, orientationLowerBound);
         }
         motionPlanner.submitNode(motionNode);
      }
      ContactState contactState = contactStateList.get(contactStateList.size() - 1);
      ContactType contactType = contactState.getContactType();
      double contactStateDuration = contactState.getDuration();
      contactState.getSupportPolygon(tempPolygon);
      contactState.getCoMPosition(tempCentroid);
      positionUpperBound.setIncludingFrame(tempCentroid);
      positionUpperBound.add(positionUpperBoundDelta);
      positionLowerBound.setIncludingFrame(tempCentroid);
      positionLowerBound.add(positionLowerBoundDelta);
      
      contactState.getCoMOrientation(tempOrientation);
      tempOrientationVector.setIncludingFrame(tempOrientation.getReferenceFrame(), tempOrientation.getRoll(), tempOrientation.getPitch(),
                                              tempOrientation.getYaw());
      orientationUpperBound.setIncludingFrame(tempOrientationVector);
      orientationUpperBound.add(orientationUpperBoundDelta);
      orientationLowerBound.setIncludingFrame(tempOrientationVector);
      orientationLowerBound.add(orientationLowerBoundDelta);

      int numberOfNodesInState = (int) Math.ceil(contactStateDuration / plannerDT);
      for (int j = 0; j < numberOfNodesInState - 1; j++)
      {
         nodeTime += plannerDT;
         motionNode.reset();
         motionNode.setTime(nodeTime);
         switch (contactType)
         {
         case NO_SUPPORT:
            motionNode.setZeroForceConstraint();
            motionNode.setZeroForceRateConstraint();
            motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
            motionNode.setZeroTorqueConstraint();
            motionNode.setZeroTorqueRateConstraint();
            motionNode.setOrientationInequalities(orientationUpperBound, orientationLowerBound);
            break;
         case DOUBLE_SUPPORT:
            motionNode.setForceObjective(nominalForce);
            motionNode.setForceRateObjective(nominalForceRate);
            motionNode.setPositionInequalities(positionUpperBound, positionLowerBound);
            motionNode.setTorqueObjective(nominalTorque);
            motionNode.setTorqueRateObjective(nominalTorqueRate);
            motionNode.setOrientationInequalities(orientationUpperBound, orientationLowerBound);
            break;
         default:
            throw new RuntimeException("Unknown support state. Cannot populate motion planner nodes");
         }
         motionPlanner.submitNode(motionNode);
      }
      double finalNodeDuration = contactStateDuration - (numberOfNodesInState - 1) * plannerDT;
      nodeTime += finalNodeDuration;
      motionNode.reset();
      motionNode.setTime(nodeTime);
      if (contactType == ContactType.NO_SUPPORT)
      {
         motionNode.setZeroForceConstraint();
         motionNode.setZeroForceRateConstraint();
         motionNode.setPositionObjective(finalPosition, positionWeight);
         motionNode.setLinearVelocityObjective(finalLinearVelocity, velocityWeight);
         motionNode.setZeroTorqueConstraint();
         motionNode.setZeroTorqueRateConstraint();
         motionNode.setOrientationObjective(finalOrientation, orientationWeight);
         motionNode.setAngularVelocityObjective(finalAngularVelocity, angularVelocityWeight);
      }
      else
      {
         motionNode.setForceConstraint(nominalForce);
         motionNode.setForceRateConstraint(nominalForceRate);
         motionNode.setPositionObjective(finalPosition, positionWeight, positionUpperBoundDelta, positionLowerBoundDelta);
         motionNode.setLinearVelocityObjective(finalLinearVelocity, velocityWeight);
         motionNode.setTorqueConstraint(nominalTorque);
         motionNode.setTorqueRateConstraint(nominalTorqueRate);
         motionNode.setOrientationObjective(finalOrientation, orientationWeight, orientationUpperBound, orientationLowerBound);
         motionNode.setAngularVelocityObjective(finalAngularVelocity, angularVelocityWeight);
      }
      motionPlanner.submitNode(motionNode);
   }

   public void computeMotionPlan()
   {
      motionPlanner.compute();
      //angularMotionPlanner.compute();
   }

   public ForceTrajectory getGroundReactionForceProfile()
   {
      return motionPlanner.getForceProfile();
   }

   public PositionTrajectory getPositionTrajectory()
   {
      return motionPlanner.getPositionTrajectory();
   }

   public List<CentroidalMotionNode> getPlannerNodes()
   {
      return motionPlanner.getNodeList();
   }

   public void reset()
   {
      motionPlanner.reset();
   }

   public ReferenceFrame getPlanningFrame()
   {
      return planningFrame;
   }

   public void setFinalPosition(FramePoint3D positionToSet)
   {
      this.finalPosition.setIncludingFrame(positionToSet);
      this.finalPosition.changeFrame(planningFrame);
   }

   public void getNominalState(FrameVector3D velocity, FrameVector3D groundReactionForce)
   {
      velocity.setToZero(planningFrame);
      groundReactionForce.setIncludingFrame(nominalForce);
   }

   public double getNominalHeight()
   {
      return 0.437;
   }
}
