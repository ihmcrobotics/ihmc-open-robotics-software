package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class CentroidalMomentumMatrix
{
   private final JointBasics[] jointList;
   private final ReferenceFrame centerOfMassFrame;
   private final DenseMatrix64F centroidalMomentumMatrix;

   private final Momentum[] unitMomenta;
   private final Momentum tempMomentum = new Momentum();
   private final Twist tempTwist = new Twist();
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(6, 1);
   private final Vector3D zero = new Vector3D();
   private final boolean[][] isAncestorMapping;

   public CentroidalMomentumMatrix(RigidBody rootBody, ReferenceFrame centerOfMassFrame)
   {
      this.jointList = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      this.centerOfMassFrame = centerOfMassFrame;
      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointList);
      this.centroidalMomentumMatrix = new DenseMatrix64F(6, nDegreesOfFreedom);
      this.unitMomenta = new Momentum[nDegreesOfFreedom];
      for (int i = 0; i < nDegreesOfFreedom; i++)
         unitMomenta[i] = new Momentum(centerOfMassFrame);
      
      isAncestorMapping = new boolean[jointList.length][jointList.length];

      for (int j = 0; j < jointList.length; j++)
      {
         RigidBody columnRigidBody = jointList[j].getSuccessor();
         for (int i = 0; i < jointList.length; i++)
         {
            RigidBody rowRigidBody = jointList[i].getSuccessor();
            isAncestorMapping[i][j] = ScrewTools.isAncestor(rowRigidBody, columnRigidBody);
         }
      }
   }

   public void compute()
   {
      for (Momentum momentum : unitMomenta)
      {
         momentum.getAngularPart().set(zero);
         momentum.getLinearPart().set(zero);
      }
      
      int column = 0;
      for (int j = 0; j < jointList.length; j++)
      {
         JointBasics columnJoint = jointList[j];
         int columnJointDegreesOfFreedom = columnJoint.getDegreesOfFreedom();

         for (int k = 0; k < columnJointDegreesOfFreedom; k++)
         {  
            for (int i = 0; i < jointList.length; i++)
            {
               if (isAncestorMapping[i][j])
               {
                  RigidBody rowRigidBody = jointList[i].getSuccessor();
                  SpatialInertiaBasics inertia = rowRigidBody.getInertia();
                  tempTwist.setIncludingFrame(columnJoint.getUnitTwists().get(k));
                  tempTwist.changeFrame(inertia.getReferenceFrame());
                  tempMomentum.setReferenceFrame(inertia.getReferenceFrame());
                  tempMomentum.compute(inertia, tempTwist);
                  tempMomentum.changeFrame(centerOfMassFrame);
                  unitMomenta[column].add(tempMomentum);
               }
            }
            column++;
         }
      }

      for (int i = 0; i < unitMomenta.length; i++)
      {
         Momentum unitMomentum = unitMomenta[i];
         unitMomentum.get(tempMatrix);
         MatrixTools.setMatrixBlock(centroidalMomentumMatrix, 0, i, tempMatrix, 0, 0, 6, 1, 1.0);
      }
   }

   public DenseMatrix64F getMatrix()
   {
      return centroidalMomentumMatrix;
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      return centerOfMassFrame;
   }
}
