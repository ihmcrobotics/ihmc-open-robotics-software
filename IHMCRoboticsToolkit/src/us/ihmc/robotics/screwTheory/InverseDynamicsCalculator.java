package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Computes joint torques based on desired joint accelerations.
 * Uses a recursive Newton-Euler algorithm, as described in Featherstone - Rigid Body Dynamics Algorithms (2008)
 *
 * @author Twan Koolen
 *
 */
public class InverseDynamicsCalculator
{
   private final RigidBody rootBody;
   private final ArrayList<RigidBody> listOfBodiesWithExternalWrenches = new ArrayList<>();
   private final LinkedHashMap<RigidBody, Wrench> externalWrenches;
   private final ArrayList<InverseDynamicsJoint> jointsToIgnore;

   private final ArrayList<RigidBody> allBodiesExceptRoot = new ArrayList<RigidBody>();
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
   private final LinkedHashMap<RigidBody, Wrench> netWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final LinkedHashMap<InverseDynamicsJoint, Wrench> jointWrenches = new LinkedHashMap<InverseDynamicsJoint, Wrench>();
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final SpatialAccelerationVector tempAcceleration = new SpatialAccelerationVector();
   private final Twist tempTwist = new Twist();

   private final boolean doVelocityTerms;

   private InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener;
   
   public InverseDynamicsCalculator(TwistCalculator twistCalculator, double gravity)
   {
      this(twistCalculator, gravity, new ArrayList<InverseDynamicsJoint>());
   }

   public InverseDynamicsCalculator(TwistCalculator twistCalculator, double gravity, List<InverseDynamicsJoint> jointsToIgnore)
   {
      this(ReferenceFrame.getWorldFrame(), ScrewTools.createGravitationalSpatialAcceleration(twistCalculator.getRootBody(), gravity),
            new LinkedHashMap<RigidBody, Wrench>(), jointsToIgnore, true, true, twistCalculator);
   }

   // FIXME: doVelocityTerms = false does not seem to work
   public InverseDynamicsCalculator(ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration, HashMap<RigidBody, Wrench> externalWrenches,
         List<InverseDynamicsJoint> jointsToIgnore, boolean doVelocityTerms, boolean doAccelerationTerms, TwistCalculator twistCalculator)
   {
      this(inertialFrame, externalWrenches, jointsToIgnore, new SpatialAccelerationCalculator(twistCalculator.getRootBody(), inertialFrame, rootAcceleration,
            twistCalculator, doVelocityTerms, doAccelerationTerms, true), twistCalculator);
   }

   public InverseDynamicsCalculator(ReferenceFrame inertialFrame, HashMap<RigidBody, Wrench> externalWrenches, List<InverseDynamicsJoint> jointsToIgnore,
         SpatialAccelerationCalculator spatialAccelerationCalculator, TwistCalculator twistCalculator)
   {
      this.rootBody = twistCalculator.getRootBody();
      this.externalWrenches = new LinkedHashMap<RigidBody, Wrench>(externalWrenches);
      this.jointsToIgnore = new ArrayList<InverseDynamicsJoint>(jointsToIgnore);
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      this.doVelocityTerms = spatialAccelerationCalculator.areVelocitiesConsidered();

      populateMapsAndLists();

   }

   public void setRootAcceleration(SpatialAccelerationVector newRootAcceleration)
   {
      spatialAccelerationCalculator.setRootAcceleration(newRootAcceleration);
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      if (this.inverseDynamicsCalculatorListener != null) 
      {
         throw new RuntimeException("Can only be one InverseDynamicsCalculatorListener");
      }
      
      this.inverseDynamicsCalculatorListener = inverseDynamicsCalculatorListener;
   }
   
   public void compute()
   {
      computeTwistsAndSpatialAccelerations();
      computeNetWrenches();
      computeJointWrenchesAndTorques();
      
      if (inverseDynamicsCalculatorListener != null) inverseDynamicsCalculatorListener.inverseDynamicsCalculatorIsDone(this);
   }

   public void setExternalWrench(RigidBody rigidBody, Wrench externalWrench)
   {
      externalWrenches.get(rigidBody).set(externalWrench);
   }
   
   public void getExternalWrench(RigidBody rigidBody, Wrench externalWrenchToPack)
   {
      externalWrenchToPack.set(externalWrenches.get(rigidBody));
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   private void computeTwistsAndSpatialAccelerations()
   {
      spatialAccelerationCalculator.compute();
   }

   private void computeNetWrenches()
   {
      for (int bodyIndex = 0; bodyIndex < allBodiesExceptRoot.size(); bodyIndex++)
      {
         RigidBody body = allBodiesExceptRoot.get(bodyIndex);
         Wrench netWrench = netWrenches.get(body);
         twistCalculator.getTwistOfBody(body, tempTwist);
         if (!doVelocityTerms)
            tempTwist.setToZero();
         spatialAccelerationCalculator.getAccelerationOfBody(body, tempAcceleration);
         body.getInertia().computeDynamicWrenchInBodyCoordinates(tempAcceleration, tempTwist, netWrench);
      }
   }

   private final Wrench wrenchExertedByChild = new Wrench();

   private void computeJointWrenchesAndTorques()
   {
      for (int jointIndex = allJoints.size() - 1; jointIndex >= 0; jointIndex--)
      {
         InverseDynamicsJoint joint = allJoints.get(jointIndex);

         RigidBody successor = joint.getSuccessor();

         Wrench jointWrench = jointWrenches.get(joint);
         jointWrench.set(netWrenches.get(successor));

         Wrench externalWrench = externalWrenches.get(successor);
         jointWrench.sub(externalWrench);

         List<InverseDynamicsJoint> childrenJoints = successor.getChildrenJoints();

         for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
         {
            InverseDynamicsJoint child = childrenJoints.get(childIndex);
            if (!jointsToIgnore.contains(child))
            {
               Wrench wrenchExertedOnChild = jointWrenches.get(child);
               ReferenceFrame successorFrame = successor.getBodyFixedFrame();

               wrenchExertedByChild.set(wrenchExertedOnChild);
               wrenchExertedByChild.changeBodyFrameAttachedToSameBody(successorFrame);
               wrenchExertedByChild.scale(-1.0); // Action = -reaction
               wrenchExertedByChild.changeFrame(jointWrench.getExpressedInFrame());
               jointWrench.sub(wrenchExertedByChild);
            }
         }

         joint.setTorqueFromWrench(jointWrench);
      }
   }

   private void populateMapsAndLists()
   {
      ArrayList<RigidBody> morgue = new ArrayList<RigidBody>();
      morgue.add(rootBody);

      while (!morgue.isEmpty())
      {
         RigidBody currentBody = morgue.get(0);

         ReferenceFrame bodyFixedFrame = currentBody.getBodyFixedFrame();

         if (!currentBody.isRootBody())
         {
            allBodiesExceptRoot.add(currentBody);
            netWrenches.put(currentBody, new Wrench(bodyFixedFrame, bodyFixedFrame));
            if (externalWrenches.get(currentBody) == null)
            {
               listOfBodiesWithExternalWrenches.add(currentBody);
               externalWrenches.put(currentBody, new Wrench(bodyFixedFrame, bodyFixedFrame));
            }
         }

         if (currentBody.hasChildrenJoints())
         {
            List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
            for (InverseDynamicsJoint joint : childrenJoints)
            {
               if (!jointsToIgnore.contains(joint))
               {
                  RigidBody successor = joint.getSuccessor();
                  if (successor != null)
                  {
                     if (allBodiesExceptRoot.contains(successor))
                     {
                        throw new RuntimeException("This algorithm doesn't do loops.");
                     }

                     allJoints.add(joint);
                     jointWrenches.put(joint, new Wrench());
                     morgue.add(successor);
                  }
               }
            }
         }

         morgue.remove(currentBody);
      }
   }

   public void reset()
   {
      for (int i = 0; i < listOfBodiesWithExternalWrenches.size(); i++)
      {
         Wrench externalWrench = externalWrenches.get(listOfBodiesWithExternalWrenches.get(i));
         externalWrench.setToZero(externalWrench.getBodyFrame(), externalWrench.getExpressedInFrame());
      }
   }

   public Wrench computeTotalExternalWrench(ReferenceFrame referenceFrame)
   {
      Wrench totalGroundReactionWrench = new Wrench(referenceFrame, referenceFrame);
      Wrench temporaryWrench = new Wrench();
      for (int i = 0; i < listOfBodiesWithExternalWrenches.size(); i++)
      {
         Wrench externalWrench = externalWrenches.get(listOfBodiesWithExternalWrenches.get(i));
         temporaryWrench.set(externalWrench);
         temporaryWrench.changeFrame(referenceFrame);
         temporaryWrench.changeBodyFrameAttachedToSameBody(referenceFrame);
         totalGroundReactionWrench.add(temporaryWrench);
      }

      return totalGroundReactionWrench;
   }

   public void getJointWrench(InverseDynamicsJoint joint, Wrench wrenchToPack)
   {
      wrenchToPack.set(jointWrenches.get(joint));
   }
}
