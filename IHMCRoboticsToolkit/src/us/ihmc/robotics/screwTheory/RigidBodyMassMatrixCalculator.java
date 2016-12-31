package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

public class RigidBodyMassMatrixCalculator
{
   private final RigidBody rootBody;
   private final ArrayList<RigidBody> listOfBodiesWithExternalWrenches = new ArrayList<>();
   private final LinkedHashMap<RigidBody, Wrench> externalWrenches;
   private final ArrayList<InverseDynamicsJoint> jointsToIgnore;

   private final ArrayList<RigidBody> allBodiesExceptRoot = new ArrayList<RigidBody>();
   private final ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
   private final LinkedHashMap<RigidBody, Wrench> netWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final LinkedHashMap<InverseDynamicsJoint, Wrench> jointWrenches = new LinkedHashMap<InverseDynamicsJoint, Wrench>();
   private final LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F> coriolisWrenches = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final SpatialAccelerationVector tempAcceleration = new SpatialAccelerationVector();
   private final Twist tempTwist = new Twist();

   private final int degreesOfFreedom;

   public RigidBodyMassMatrixCalculator(TwistCalculator twistCalculator, List<InverseDynamicsJoint> jointsToIgnore)
   {
      this(ReferenceFrame.getWorldFrame(), ScrewTools.createGravitationalSpatialAcceleration(twistCalculator.getRootBody(), 0.0),
            new LinkedHashMap<>(), jointsToIgnore, twistCalculator);
   }

   public RigidBodyMassMatrixCalculator(ReferenceFrame inertialFrame, SpatialAccelerationVector rootAcceleration, HashMap<RigidBody, Wrench> externalWrenches,
         List<InverseDynamicsJoint> jointsToIgnore,  TwistCalculator twistCalculator)
   {
      this(externalWrenches, jointsToIgnore, new SpatialAccelerationCalculator(twistCalculator.getRootBody(), inertialFrame, rootAcceleration,
            twistCalculator, false, true, true), twistCalculator);
   }

   //// TODO: 12/31/16  remove explicit dependency on the spatial acceleration calculator
   public RigidBodyMassMatrixCalculator(HashMap<RigidBody, Wrench> externalWrenches, List<InverseDynamicsJoint> jointsToIgnore,
         SpatialAccelerationCalculator spatialAccelerationCalculator, TwistCalculator twistCalculator)
   {
      this.rootBody = twistCalculator.getRootBody();
      this.externalWrenches = new LinkedHashMap<>(externalWrenches);
      this.jointsToIgnore = new ArrayList<>(jointsToIgnore);
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      populateMapsAndLists();

      degreesOfFreedom = ScrewTools.computeDegreesOfFreedom(allJoints);
   }

   public void setRootAcceleration(SpatialAccelerationVector newRootAcceleration)
   {
      spatialAccelerationCalculator.setRootAcceleration(newRootAcceleration);
   }

   public void compute()
   {
      computeTwistsAndSpatialAccelerations();
      computeNetWrenches();
      computeJointWrenchesAndTorques();
   }


   public void setExternalWrench(RigidBody rigidBody, Wrench externalWrench)
   {
      externalWrenches.get(rigidBody).set(externalWrench);
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   private void zeroAccelerations()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         InverseDynamicsJoint joint = allJoints.get(i);
         joint.setDesiredAccelerationToZero();
      }
   }

   private void computeTwistsAndSpatialAccelerations()
   {
      zeroAccelerations();
      spatialAccelerationCalculator.compute();
   }

   private void computeNetWrenches()
   {
      for (int bodyIndex = 0; bodyIndex < allBodiesExceptRoot.size(); bodyIndex++)
      {
         RigidBody body = allBodiesExceptRoot.get(bodyIndex);
         Wrench netWrench = netWrenches.get(body);
         twistCalculator.getTwistOfBody(tempTwist, body);
         tempTwist.setToZero();
         spatialAccelerationCalculator.getAccelerationOfBody(tempAcceleration, body);
         body.getInertia().computeDynamicWrenchInBodyCoordinates(netWrench, tempAcceleration, tempTwist);
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

         DenseMatrix64F coriolisWrench = coriolisWrenches.get(joint);
         coriolisWrench.zero();
         joint.getTauMatrix(coriolisWrench);
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
                     coriolisWrenches.put(joint, new DenseMatrix64F(joint.getDegreesOfFreedom(), 1));
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

   public int getNumberOfDegreesOfFreedom()
   {
      return degreesOfFreedom;
   }

   public void getJointCoriolisMatrix(InverseDynamicsJoint joint, DenseMatrix64F jointCoriolisMatrixToPack)
   {
      jointCoriolisMatrixToPack.set(coriolisWrenches.get(joint));
   }
}
