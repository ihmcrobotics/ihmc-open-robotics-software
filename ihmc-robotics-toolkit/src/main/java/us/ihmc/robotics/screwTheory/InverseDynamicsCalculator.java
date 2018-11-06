package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

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
   private final List<RigidBody> listOfBodiesWithExternalWrenches = new ArrayList<>();
   private final Map<RigidBody, Wrench> externalWrenches = new LinkedHashMap<>();
   private final List<JointBasics> jointsToIgnore;

   private final List<RigidBody> allBodiesExceptRoot = new ArrayList<RigidBody>();
   private final List<JointBasics> allJoints = new ArrayList<JointBasics>();
   private final Map<RigidBody, Wrench> netWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final Map<JointBasics, Wrench> jointWrenches = new LinkedHashMap<JointBasics, Wrench>();
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final SpatialAcceleration tempAcceleration = new SpatialAcceleration();
   private final Twist tempTwist = new Twist();

   private final boolean doVelocityTerms;

   private InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener;
   
   public InverseDynamicsCalculator(RigidBody body, double gravity)
   {
      this(body, gravity, new ArrayList<JointBasics>());
   }

   public InverseDynamicsCalculator(RigidBody body, double gravity, List<JointBasics> jointsToIgnore)
   {
      this(body, ScrewTools.createGravitationalSpatialAcceleration(ScrewTools.getRootBody(body), gravity),
            jointsToIgnore, true, true);
   }

   // FIXME: doVelocityTerms = false does not seem to work
   public InverseDynamicsCalculator(RigidBody body, SpatialAccelerationReadOnly rootAcceleration, List<JointBasics> jointsToIgnore,
         boolean doVelocityTerms, boolean doAccelerationTerms)
   {
      this(jointsToIgnore, new SpatialAccelerationCalculator(body, rootAcceleration, doVelocityTerms,
            doAccelerationTerms, true));
   }

   public InverseDynamicsCalculator(List<JointBasics> jointsToIgnore, SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.rootBody = spatialAccelerationCalculator.getRootBody();
      this.jointsToIgnore = new ArrayList<JointBasics>(jointsToIgnore);
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      this.doVelocityTerms = spatialAccelerationCalculator.areVelocitiesConsidered();

      populateMapsAndLists();

   }

   public void setRootAcceleration(SpatialAccelerationReadOnly newRootAcceleration)
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

   public void setExternalWrench(RigidBody rigidBody, WrenchReadOnly externalWrench)
   {
      externalWrenches.get(rigidBody).setIncludingFrame(externalWrench);
   }
   
   public void getExternalWrench(RigidBody rigidBody, Wrench externalWrenchToPack)
   {
      externalWrenchToPack.setIncludingFrame(externalWrenches.get(rigidBody));
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
         body.getBodyFixedFrame().getTwistOfFrame(tempTwist);
         if (!doVelocityTerms)
            tempTwist.setToZero();
         spatialAccelerationCalculator.getAccelerationOfBody(body, tempAcceleration);
         body.getInertia().computeDynamicWrenchFast(tempAcceleration, tempTwist, netWrench);
      }
   }

   private final Wrench wrenchExertedByChild = new Wrench();

   private void computeJointWrenchesAndTorques()
   {
      for (int jointIndex = allJoints.size() - 1; jointIndex >= 0; jointIndex--)
      {
         JointBasics joint = allJoints.get(jointIndex);

         RigidBody successor = joint.getSuccessor();

         Wrench jointWrench = jointWrenches.get(joint);
         jointWrench.setIncludingFrame(netWrenches.get(successor));

         WrenchReadOnly externalWrench = externalWrenches.get(successor);
         jointWrench.sub(externalWrench);

         List<JointBasics> childrenJoints = successor.getChildrenJoints();

         for (int childIndex = 0; childIndex < childrenJoints.size(); childIndex++)
         {
            JointBasics child = childrenJoints.get(childIndex);
            if (!jointsToIgnore.contains(child))
            {
               WrenchReadOnly wrenchExertedOnChild = jointWrenches.get(child);
               ReferenceFrame successorFrame = successor.getBodyFixedFrame();

               wrenchExertedByChild.setIncludingFrame(wrenchExertedOnChild);
               wrenchExertedByChild.setBodyFrame(successorFrame);
               wrenchExertedByChild.scale(-1.0); // Action = -reaction
               wrenchExertedByChild.changeFrame(jointWrench.getReferenceFrame());
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
            List<JointBasics> childrenJoints = currentBody.getChildrenJoints();
            for (JointBasics joint : childrenJoints)
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
         externalWrench.setToZero(externalWrench.getBodyFrame(), externalWrench.getReferenceFrame());
      }
   }

   public WrenchReadOnly computeTotalExternalWrench(ReferenceFrame referenceFrame)
   {
      Wrench totalGroundReactionWrench = new Wrench(referenceFrame, referenceFrame);
      Wrench temporaryWrench = new Wrench();
      for (int i = 0; i < listOfBodiesWithExternalWrenches.size(); i++)
      {
         WrenchReadOnly externalWrench = externalWrenches.get(listOfBodiesWithExternalWrenches.get(i));
         temporaryWrench.setIncludingFrame(externalWrench);
         temporaryWrench.changeFrame(referenceFrame);
         temporaryWrench.setBodyFrame(referenceFrame);
         totalGroundReactionWrench.add(temporaryWrench);
      }

      return totalGroundReactionWrench;
   }

   public void getJointWrench(JointBasics joint, Wrench wrenchToPack)
   {
      wrenchToPack.setIncludingFrame(jointWrenches.get(joint));
   }
}
