package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.LinkedHashMap;

public class SpatialAccelerationCalculator
{
   private final RigidBody rootBody;
   private final SpatialAccelerationVector rootAcceleration;
   private final TwistCalculator twistCalculator;
   private final boolean doVelocityTerms;
   private final boolean doAccelerationTerms;
   private final boolean useDesireds;

   private final LinkedHashMap<RigidBody, SpatialAccelerationVector> accelerations = new LinkedHashMap<RigidBody, SpatialAccelerationVector>();
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
   private final Twist tempJointTwist = new Twist();
   private final Twist tempTwistFromWorld = new Twist();
   private final SpatialAccelerationVector tempJointAcceleration = new SpatialAccelerationVector();
   private final ReferenceFrame inertialFrame;

   public SpatialAccelerationCalculator(RigidBody body, ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration,
           TwistCalculator twistCalculator, boolean doVelocityTerms, boolean useDesireds)
   {
      this(body, inertialFrame, rootAcceleration, twistCalculator, doVelocityTerms, true, useDesireds);
   }
   
   public SpatialAccelerationCalculator(RigidBody body, ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration,
           TwistCalculator twistCalculator, boolean doVelocityTerms, boolean doAccelerationTerms, boolean useDesireds)
   {
      this.rootBody = addAllPrecedingJoints(inertialFrame, body);
      this.rootAcceleration = new SpatialAccelerationVector(rootAcceleration);
      this.twistCalculator = twistCalculator;
      this.doVelocityTerms = doVelocityTerms;
      this.doAccelerationTerms = doAccelerationTerms;
      this.useDesireds = useDesireds;
      this.inertialFrame = inertialFrame;

      addAllSuccedingJoints(inertialFrame, body);
      populateMapsAndLists();
   }

   public void setRootAcceleration(SpatialAccelerationVector newRootAcceleration)
   {
      rootAcceleration.checkReferenceFramesMatch(newRootAcceleration.getBodyFrame(), newRootAcceleration.getBaseFrame(), newRootAcceleration.getExpressedInFrame());
      this.rootAcceleration.set(newRootAcceleration);
   }
   
   private void addAllSuccedingJoints(ReferenceFrame inertialFrame, RigidBody body)
   {
      for (InverseDynamicsJoint inverseDynamicsJoint : body.getChildrenJoints())
      {
         if (inverseDynamicsJoint.getSuccessor() != null)
         {
            allJoints.add(inverseDynamicsJoint);
            addAllSuccedingJoints(inertialFrame, inverseDynamicsJoint.getSuccessor());
         }
      }
   }

   private RigidBody addAllPrecedingJoints(ReferenceFrame inertialFrame, RigidBody body)
   {
      ArrayList<InverseDynamicsJoint> jointsTillRootBody = new ArrayList<InverseDynamicsJoint>();

      RigidBody currentBody = body;
      while (currentBody.getParentJoint() != null)
      {
         jointsTillRootBody.add(currentBody.getParentJoint());
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      for (int i = jointsTillRootBody.size() - 1; i >= 0; i--)
      {
         allJoints.add(jointsTillRootBody.get(i));
      }

      return currentBody;
   }

   public SpatialAccelerationCalculator(RigidBody rootBody, TwistCalculator twistCalculator, double gravity, boolean useDesireds)
   {
      this(rootBody, ReferenceFrame.getWorldFrame(), ScrewTools.createGravitationalSpatialAcceleration(rootBody, gravity), twistCalculator, true, useDesireds);
   }

   public void compute()
   {
      accelerations.get(rootBody).set(rootAcceleration);

      for (int jointIndex = 0; jointIndex < allJoints.size(); jointIndex++)
      {
         InverseDynamicsJoint joint = allJoints.get(jointIndex);
         computeSuccessorAcceleration(joint);
      }
   }

   private void computeSuccessorAcceleration(InverseDynamicsJoint joint)
   {
      RigidBody predecessor = joint.getPredecessor();
      RigidBody successor = joint.getSuccessor();
      ReferenceFrame successorFrame = successor.getBodyFixedFrame();
      joint.getPredecessorTwist(tempJointTwist);
      if (!doVelocityTerms)
         tempJointTwist.setToZero();

      if (useDesireds)
         joint.getDesiredSuccessorAcceleration(tempJointAcceleration);
      else
         joint.getSuccessorAcceleration(tempJointAcceleration);
      if (!doAccelerationTerms)
         tempJointAcceleration.setToZero();
      
      twistCalculator.packTwistOfBody(tempTwistFromWorld, predecessor);
      if (!doVelocityTerms)
         tempTwistFromWorld.setToZero();
      SpatialAccelerationVector successorAcceleration = accelerations.get(successor);
      successorAcceleration.set(accelerations.get(predecessor));
      successorAcceleration.changeFrame(successorFrame, tempJointTwist, tempTwistFromWorld);
      successorAcceleration.add(tempJointAcceleration);
   }

   public void packAccelerationOfBody(SpatialAccelerationVector spatialAccelerationToPack, RigidBody rigidBody)
   {
      spatialAccelerationToPack.set(accelerations.get(rigidBody));
   }

   private final Twist twistOfCurrentWithRespectToNew = new Twist();
   private final Twist twistOfBodyWithRespectToBase = new Twist();
   private final SpatialAccelerationVector baseAcceleration = new SpatialAccelerationVector();
   public void packRelativeAcceleration(final SpatialAccelerationVector endEffectorAcceleration, RigidBody base, RigidBody body)
   {
      twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, body, base);
      twistOfCurrentWithRespectToNew.changeFrame(base.getBodyFixedFrame());
      
      twistCalculator.packTwistOfBody(twistOfBodyWithRespectToBase, base);

      packAccelerationOfBody(baseAcceleration, base);
      packAccelerationOfBody(endEffectorAcceleration, body);
      
      baseAcceleration.changeFrame(endEffectorAcceleration.getExpressedInFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
      endEffectorAcceleration.sub(baseAcceleration);
   }
   
   private final SpatialAccelerationVector endEffectorAcceleration = new SpatialAccelerationVector();
   public void packLinearAccelerationOfBodyFixedPoint(FrameVector linearAccelerationToPack, RigidBody base, RigidBody body, FramePoint bodyFixedPoint)
   {
      twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, base, body);
      twistCalculator.packTwistOfBody(twistOfBodyWithRespectToBase, body);

      packAccelerationOfBody(baseAcceleration, base);
      packAccelerationOfBody(endEffectorAcceleration, body);
      
      endEffectorAcceleration.changeFrame(baseAcceleration.getBodyFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);
      endEffectorAcceleration.sub(baseAcceleration);
      bodyFixedPoint.changeFrame(endEffectorAcceleration.getExpressedInFrame());

      twistOfCurrentWithRespectToNew.changeFrame(endEffectorAcceleration.getExpressedInFrame());
      endEffectorAcceleration.packAccelerationOfPointFixedInBodyFrame(twistOfCurrentWithRespectToNew, bodyFixedPoint, linearAccelerationToPack);
   }
   
   public void packLinearAccelerationOfBodyFixedPoint(FrameVector linearAccelerationToPack, RigidBody body, FramePoint bodyFixedPoint)
   {
      packLinearAccelerationOfBodyFixedPoint(linearAccelerationToPack, getRootBody(), body, bodyFixedPoint);
   }
   
   public RigidBody getRootBody()
   {
      return rootBody;
   }

   
   private void populateMapsAndLists()
   {
      accelerations.put(rootBody, new SpatialAccelerationVector());
      
      for (InverseDynamicsJoint joint : allJoints)
      {
         if (joint.getSuccessor() != null)
         {
            accelerations.put(joint.getSuccessor(), new SpatialAccelerationVector());
         }
      }
   }

   public ReferenceFrame getInertialFrame()
   {
      return inertialFrame;
   }
}
