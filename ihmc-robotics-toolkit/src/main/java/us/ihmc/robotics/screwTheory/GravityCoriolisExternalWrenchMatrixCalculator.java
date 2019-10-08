package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class GravityCoriolisExternalWrenchMatrixCalculator
{
   private final RigidBodyBasics rootBody;

   private final ArrayList<JointBasics> jointsToIgnore;
   private final ArrayList<JointBasics> allJoints = new ArrayList<>();
   private final ArrayList<RigidBodyBasics> allBodiesExceptRoot = new ArrayList<>();
   private final ArrayList<RigidBodyBasics> listOfBodiesWithExternalWrenches = new ArrayList<>();

   private final LinkedHashMap<RigidBodyBasics, Wrench> externalWrenches;
   private final LinkedHashMap<RigidBodyBasics, Wrench> netWrenches = new LinkedHashMap<>();
   private final LinkedHashMap<JointBasics, Wrench> jointWrenches = new LinkedHashMap<>();
   private final LinkedHashMap<JointBasics, DenseMatrix64F> coriolisWrenches = new LinkedHashMap<>();

   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private final SpatialAcceleration tempAcceleration = new SpatialAcceleration();
   private final Twist tempTwist = new Twist();

   private final boolean doVelocityTerms;

   private static final boolean DEFAULT_DO_VELOCITY_TERMS = true;
   private static final boolean DO_ACCELERATION_TERMS = false;
   private static final boolean USE_DESIRED_ACCELERATIONS = true;

   public GravityCoriolisExternalWrenchMatrixCalculator(RigidBodyBasics body, ArrayList<JointBasics> jointsToIgnore, double gravity)
   {
      this(body,ScrewTools.createGravitationalSpatialAcceleration(MultiBodySystemTools.getRootBody(body), gravity), new LinkedHashMap<>(), jointsToIgnore,
           DEFAULT_DO_VELOCITY_TERMS, DO_ACCELERATION_TERMS);
   }

   public GravityCoriolisExternalWrenchMatrixCalculator(RigidBodyBasics body, SpatialAccelerationReadOnly rootAcceleration, HashMap<RigidBodyBasics, Wrench> externalWrenches,
                                                        ArrayList<JointBasics> jointsToIgnore, boolean doVelocityTerms, boolean doAccelerationTerms)
   {
      this(externalWrenches, jointsToIgnore, createSpatialAccelerationCalculator(body, rootAcceleration, doVelocityTerms, doAccelerationTerms));
   }

   //// TODO: 12/31/16  remove explicit dependency on the spatial acceleration calculator
   public GravityCoriolisExternalWrenchMatrixCalculator(HashMap<RigidBodyBasics, Wrench> externalWrenches, List<JointBasics> jointsToIgnore,
                                                        SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this.rootBody = (RigidBodyBasics) spatialAccelerationCalculator.getRootBody();
      this.externalWrenches = new LinkedHashMap<>(externalWrenches);
      this.jointsToIgnore = new ArrayList<>(jointsToIgnore);
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      this.doVelocityTerms = spatialAccelerationCalculator.areVelocitiesConsidered();

      populateMapsAndLists();
   }

   private static SpatialAccelerationCalculator createSpatialAccelerationCalculator(RigidBodyReadOnly body, SpatialAccelerationReadOnly rootAcceleration,
                                                                                           boolean doVelocityTerms, boolean doAccelerationTerms)
   {
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(body, ReferenceFrame.getWorldFrame(), doVelocityTerms, doAccelerationTerms);
      spatialAccelerationCalculator.setRootAcceleration(rootAcceleration);
      return spatialAccelerationCalculator;
   }

   public void setRootAcceleration(SpatialAccelerationReadOnly newRootAcceleration)
   {
      spatialAccelerationCalculator.setRootAcceleration(newRootAcceleration);
   }

   public void compute()
   {
      computeTwistsAndSpatialAccelerations();
      computeNetWrenches();
      computeJointWrenchesAndTorques();
   }

   public void setExternalWrench(RigidBodyBasics rigidBody, WrenchReadOnly externalWrench)
   {
      externalWrenches.get(rigidBody).setIncludingFrame(externalWrench);
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   private void computeTwistsAndSpatialAccelerations()
   {
      spatialAccelerationCalculator.reset();
   }

   private void computeNetWrenches()
   {
      for (int bodyIndex = 0; bodyIndex < allBodiesExceptRoot.size(); bodyIndex++)
      {
         RigidBodyBasics body = allBodiesExceptRoot.get(bodyIndex);
         Wrench netWrench = netWrenches.get(body);
         body.getBodyFixedFrame().getTwistOfFrame(tempTwist);
         if (!doVelocityTerms)
            tempTwist.setToZero();
         tempAcceleration.setIncludingFrame(spatialAccelerationCalculator.getAccelerationOfBody(body));
         body.getInertia().computeDynamicWrenchFast(tempAcceleration, tempTwist, netWrench);
      }
   }

   private final Wrench wrenchExertedByChild = new Wrench();
   private void computeJointWrenchesAndTorques()
   {
      for (int jointIndex = allJoints.size() - 1; jointIndex >= 0; jointIndex--)
      {
         JointBasics joint = allJoints.get(jointIndex);

         RigidBodyBasics successor = joint.getSuccessor();

         Wrench jointWrench = jointWrenches.get(joint);
         jointWrench.setIncludingFrame(netWrenches.get(successor));

         WrenchReadOnly externalWrench = externalWrenches.get(successor);
         jointWrench.sub(externalWrench);

         List<? extends JointBasics> childrenJoints = successor.getChildrenJoints();

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

         jointWrench.changeFrame(joint.getFrameAfterJoint());
         joint.setJointWrench(jointWrench);

         DenseMatrix64F coriolisWrench = coriolisWrenches.get(joint);
         coriolisWrench.zero();
         joint.getJointTau(0, coriolisWrench);
      }
   }

   private void populateMapsAndLists()
   {
      ArrayList<RigidBodyBasics> morgue = new ArrayList<RigidBodyBasics>();
      morgue.add(rootBody);

      while (!morgue.isEmpty())
      {
         RigidBodyBasics currentBody = morgue.get(0);

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
            List<? extends JointBasics> childrenJoints = currentBody.getChildrenJoints();
            for (JointBasics joint : childrenJoints)
            {
               if (!jointsToIgnore.contains(joint))
               {
                  RigidBodyBasics successor = joint.getSuccessor();
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
         externalWrench.setToZero(externalWrench.getBodyFrame(), externalWrench.getReferenceFrame());
      }
   }

   public void getJointCoriolisMatrix(JointBasics joint, DenseMatrix64F jointCoriolisMatrixToPack)
   {
      jointCoriolisMatrixToPack.set(coriolisWrenches.get(joint));
   }

   public void getJointCoriolisMatrix(JointBasics joint, DenseMatrix64F jointCoriolisMatrixToPack, int startRow)
   {
      DenseMatrix64F coriolisWrench = coriolisWrenches.get(joint);
      CommonOps.extract(coriolisWrench, 0, coriolisWrench.getNumRows(), 0, 1, jointCoriolisMatrixToPack, startRow, 0);
   }
}
