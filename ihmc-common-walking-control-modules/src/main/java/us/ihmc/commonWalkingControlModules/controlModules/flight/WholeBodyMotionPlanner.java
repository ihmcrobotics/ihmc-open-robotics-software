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

   private final double gravityZ;
   private final double robotMass;
   private final double plannerDT;

   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempOrientationVector = new FrameVector3D();
   private final FrameVector3D initialOrientationVector = new FrameVector3D();
   private final FrameVector3D finalOrientationVector = new FrameVector3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();
   private final FrameVector3D initialGroundReactionForce = new FrameVector3D();
   private final FrameVector3D finalGroundReactionForce = new FrameVector3D();
   private final FrameVector3D initialTorque = new FrameVector3D();
   private final FrameVector3D finalTorque = new FrameVector3D();
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
   private final FrameVector3D linearVelocityWeight;
   private final FrameVector3D orientationWeight;
   private final FrameVector3D angularVelocityWeight;

   public WholeBodyMotionPlanner(CentroidalMotionPlannerParameters parameters, YoVariableRegistry registry)
   {
      this.plannerDT = parameters.getDeltaTMin();
      this.gravityZ = parameters.getGravityZ();
      this.robotMass = parameters.getRobotMass();
      this.motionPlanner = new CentroidalMotionPlanner(parameters, registry);
      this.motionNode = new CentroidalMotionNode(planningFrame);

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
      this.linearVelocityWeight = new FrameVector3D(planningFrame, defaultLinearVelocityWeight, defaultLinearVelocityWeight, defaultLinearVelocityWeight);

      double defaultOrientationWeight = parameters.getDefaultMotionPlanningOrientationObjectiveWeight();
      this.orientationWeight = new FrameVector3D(planningFrame, defaultOrientationWeight, defaultOrientationWeight, defaultOrientationWeight);
      double defaultAngularVelocityWeight = parameters.getDefaultMotionPlanningAngularVelocityObjectiveWeight();
      this.angularVelocityWeight = new FrameVector3D(planningFrame, defaultAngularVelocityWeight, defaultAngularVelocityWeight, defaultAngularVelocityWeight);
   }

   public void processContactStatesAndGenerateMotionNodesForPlanning(List<ContactState> contactStateList)
   {
      if (contactStateList.size() < 1)
         return;
      double nodeTime = 0.0;
      motionNode.reset();
      motionNode.setTime(nodeTime);
      {
         ContactState contactState = contactStateList.get(0);
         motionNode.setPositionConstraint(initialPosition);
         motionNode.setLinearVelocityConstraint(initialLinearVelocity);
         motionNode.setForceConstraint(initialGroundReactionForce);
         motionNode.setZeroForceRateConstraint();
         motionNode.setOrientationConstraint(initialOrientationVector);
         motionNode.setAngularVelocityConstraint(initialAngularVelocity);
         motionNode.setTorqueConstraint(initialTorque);
         motionNode.setZeroTorqueRateConstraint();
         PrintTools.debug(motionNode.toString());
         motionPlanner.submitNode(motionNode);
      }
      for (int i = 0; i < contactStateList.size() - 1; i++)
      {
         ContactState contactState = contactStateList.get(i);
         ContactType contactType = contactState.getContactType();
         ContactType nextContactType = contactStateList.get(i + 1).getContactType();
         contactState.getSupportPolygon(tempPolygon);
         tempPosition.setIncludingFrame(contactState.getReferenceFrame(), tempPolygon.getCentroid(), 0.0);
         positionUpperBound.setIncludingFrame(tempPosition);
         positionUpperBound.add(positionUpperBoundDelta);
         positionLowerBound.setIncludingFrame(tempPosition);
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
            PrintTools.debug(motionNode.toString());
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
            if(nextContactType == ContactType.NO_SUPPORT)
            {
               positionJumpUpperBound.setIncludingFrame(tempPosition);
               positionJumpUpperBound.add(positionJumpUpperBoundDelta);
               motionNode.setPositionInequalities(positionJumpUpperBound, positionLowerBound);
            }
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
         PrintTools.debug(motionNode.toString());
         motionPlanner.submitNode(motionNode);
      }
      {
         ContactState contactState = contactStateList.get(contactStateList.size() - 1);
         ContactType contactType = contactState.getContactType();
         double contactStateDuration = contactState.getDuration();
         contactState.getSupportPolygon(tempPolygon);
         tempPosition.setIncludingFrame(contactState.getReferenceFrame(), tempPolygon.getCentroid(), 0.0);
         positionUpperBound.setIncludingFrame(tempPosition);
         positionUpperBound.add(positionUpperBoundDelta);
         positionLowerBound.setIncludingFrame(tempPosition);
         positionLowerBound.add(positionLowerBoundDelta);

         contactState.getCoMOrientation(tempOrientation);
         finalOrientationVector.setIncludingFrame(tempOrientation.getReferenceFrame(), tempOrientation.getRoll(), tempOrientation.getPitch(),
                                                 tempOrientation.getYaw());
         orientationUpperBound.setIncludingFrame(finalOrientationVector);
         orientationUpperBound.add(orientationUpperBoundDelta);
         orientationLowerBound.setIncludingFrame(finalOrientationVector);
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
            PrintTools.debug(motionNode.toString());
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
            motionNode.setLinearVelocityObjective(finalLinearVelocity, linearVelocityWeight);
            motionNode.setZeroTorqueConstraint();
            motionNode.setZeroTorqueRateConstraint();
            motionNode.setOrientationObjective(finalOrientationVector, orientationWeight);
            motionNode.setAngularVelocityObjective(finalAngularVelocity, angularVelocityWeight);
         }
         else
         {
            motionNode.setForceConstraint(nominalForce);
            motionNode.setForceRateConstraint(nominalForceRate);
            motionNode.setPositionConstraint(finalPosition); // , positionWeight, positionUpperBound, positionLowerBound);
            motionNode.setLinearVelocityConstraint(finalLinearVelocity); //, linearVelocityWeight);
            motionNode.setTorqueConstraint(nominalTorque);
            motionNode.setTorqueRateConstraint(nominalTorqueRate);
            motionNode.setOrientationObjective(finalOrientationVector, orientationWeight, orientationUpperBound, orientationLowerBound);
            motionNode.setAngularVelocityObjective(finalAngularVelocity, angularVelocityWeight);
         }
         PrintTools.debug(motionNode.toString());
         motionPlanner.submitNode(motionNode);
      }
   }

   public void computeMotionPlan()
   {
      motionPlanner.compute();
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

   public void getNominalState(FrameVector3D linearVelocity, FrameVector3D groundReactionForce, FrameVector3D angularVelocity, FrameVector3D torque)
   {
      linearVelocity.setToZero(planningFrame);
      groundReactionForce.setIncludingFrame(nominalForce);
      angularVelocity.setToZero(planningFrame);
      torque.setToZero(planningFrame);
   }

   public void setInitialState(FramePoint3D initialPositionInState, FrameVector3D initialLinearVelocityInState, FrameVector3D initialGroundReactionForceInState,
                               FrameVector3D initialOrientationInState, FrameVector3D initialAngularVelocityInState, FrameVector3D initialTorqueInState)
   {
      this.initialPosition.setIncludingFrame(initialPositionInState);
      this.initialLinearVelocity.setIncludingFrame(initialLinearVelocityInState);
      this.initialGroundReactionForce.setIncludingFrame(initialGroundReactionForceInState);
      this.initialOrientationVector.setIncludingFrame(initialOrientationInState);
      this.initialAngularVelocity.setIncludingFrame(initialAngularVelocityInState);
      this.initialTorque.setIncludingFrame(initialTorqueInState);
   }

   public void setFinalState(FramePoint3D finalPosition, FrameVector3D finalLinearVelocity, FrameVector3D finalGroundReactionForce, FrameVector3D finalOrientation, FrameVector3D finalAngularVelocity,
                             FrameVector3D finalTorque)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalLinearVelocity.setIncludingFrame(finalLinearVelocity);
      this.finalGroundReactionForce.setIncludingFrame(finalGroundReactionForce);
      this.finalOrientationVector.setIncludingFrame(finalOrientation);
      this.finalAngularVelocity.setIncludingFrame(finalAngularVelocity);
      this.finalTorque.setIncludingFrame(finalTorque);
   }
}
