package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * The centroidal momentum can be written as \dot{h} = A * \dot{v} + \dot{A} * v.
 * This class calculates both the A matrix and the \dot{A} * v term. This implementation 
 * is going for speed/efficiency, so the readability is not the best. For a more readable 
 * implementation, see CentroidalMomentumMatrix for the A matrix and CentroidalMomentumRateADotVTerm 
 * for the \dot{A} * v term. 
 */

public class CentroidalMomentumRateTermCalculator
{
   private final InverseDynamicsJoint[] jointList;
   private final ReferenceFrame centerOfMassFrame;
   private final DenseMatrix64F centroidalMomentumMatrix;
   
   private final SpatialAccelerationVector rootAcceleration;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final SpatialAccelerationVector tempSpatialAcceleration;
   private final SpatialForceVector tempSpatialForceVector;

   private final Momentum[] unitMomenta;
   private final Momentum[] bodyMomenta;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBody[] rigidBodies;
   private final RigidBody rootBody;
   private final DenseMatrix64F[] denseAdjTimesI;
   private final DenseMatrix64F[] denseInertias;
   private final Momentum leftSide = new Momentum();
   private final Twist tempTwist = new Twist();
   private final Twist comTwist;
   private final Vector3d tempVector;
   private final Twist[] bodyTwists;
   private final DenseMatrix64F tempSpatialMotionMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F tempSpatialForceMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F tempInertiaMatrix = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F tempAdjoint = new DenseMatrix64F(6,6);
   private final Matrix3d tempMatrix3d = new Matrix3d();
   private final Matrix3d tempPTilde = new Matrix3d();
   private final DenseMatrix64F v;
   private final DenseMatrix64F aDotV;
   private final Vector3d zero = new Vector3d();
   private final double robotMass;
   private final boolean[][] isAncestorMapping;
   private final TwistCalculator twistCalculator;

   public CentroidalMomentumRateTermCalculator(RigidBody rootBody, ReferenceFrame centerOfMassFrame,DenseMatrix64F v,double robotMass)
   {
      this.rootBody = rootBody;
      this.jointList = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      this.centerOfMassFrame = centerOfMassFrame;
      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointList);
      this.v = v;
      this.centroidalMomentumMatrix = new DenseMatrix64F(6, nDegreesOfFreedom);
      this.unitMomenta = new Momentum[nDegreesOfFreedom];
      
      for (int i = 0; i < nDegreesOfFreedom; i++)
      {
         unitMomenta[i] = new Momentum(centerOfMassFrame);
      }
      
      this.twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootBody);
      
      this.tempSpatialAcceleration = new SpatialAccelerationVector();
      this.rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame());
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, ReferenceFrame.getWorldFrame(), this.rootAcceleration,
            this.twistCalculator, true, false, false);
      
      this.denseInertias = new DenseMatrix64F[jointList.length];
      this.bodyMomenta = new Momentum[jointList.length];
      this.bodyTwists = new Twist[jointList.length];
      this.rigidBodies = new RigidBody[jointList.length];
      this.denseAdjTimesI = new DenseMatrix64F[jointList.length];
            
      isAncestorMapping = new boolean[jointList.length][jointList.length];

      for (int j = 0; j < jointList.length; j++)
      {
         bodyMomenta[j] = new Momentum(centerOfMassFrame);
         bodyTwists[j] = new Twist();
         rigidBodies[j] = jointList[j].getSuccessor();
         denseInertias[j] = rigidBodies[j].getInertia().toMatrix();
         denseAdjTimesI[j] = new DenseMatrix64F(6,6);
         
         RigidBody columnRigidBody = jointList[j].getSuccessor();
         for (int i = 0; i < jointList.length; i++)
         {
            RigidBody rowRigidBody = jointList[i].getSuccessor();
            isAncestorMapping[i][j] = ScrewTools.isAncestor(rowRigidBody, columnRigidBody);
         }
      }
      
      this.comTwist = new Twist(centerOfMassFrame, rootBody.getBodyFixedFrame(), centerOfMassFrame);
      comTwist.setToZero();
      this.tempVector = new Vector3d(0,0,0);
      this.aDotV = new DenseMatrix64F(6,1);
      this.robotMass = robotMass;
      if(robotMass == 0)
      {
         throw new RuntimeException("Your robot has zero mass");
      }
      this.tempMatrix3d.setZero();
      this.tempSpatialForceVector = new SpatialForceVector(centerOfMassFrame);
   }

   public void compute()
   {
    
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      
      precomputeAdjointTimesInertia();
      
      tempVector.set(0, 0, 0);
      for (Momentum momentum : unitMomenta)
      {
         momentum.setAngularPart(zero);
         momentum.setLinearPart(zero);
      }
      aDotV.zero();
      
      int column = 0;
      for (int j = 0; j < jointList.length; j++)
      {
         for (int k = 0; k < jointList[j].getDegreesOfFreedom(); k++)
         {
            
            for (int i = 0; i < jointList.length; i++)
            {
               if (isAncestorMapping[i][j])
               {
                  tempTwist.set(jointList[j].getMotionSubspace().getAllUnitTwists().get(k));
                  tempTwist.changeFrame(rigidBodies[i].getInertia().getExpressedInFrame());
                  
                  tempTwist.packMatrix(tempSpatialMotionMatrix, 0);
                  CommonOps.mult(denseAdjTimesI[i], tempSpatialMotionMatrix, tempMatrix);
                  tempSpatialForceVector.set(centerOfMassFrame, tempMatrix);
                  unitMomenta[column].add(tempSpatialForceVector);
               }
            }
            //pack column into the centroidal momentum matrix
            unitMomenta[column].packMatrix(tempMatrix);
            MatrixTools.setMatrixBlock(centroidalMomentumMatrix, 0, column, tempMatrix, 0, 0, 6, 1, 1.0);
            
            // Multiply column by joint velocity and sum to compute center of mass velocity
            tempVector.setX(tempVector.getX() + unitMomenta[column].getLinearPartX() * v.get(column));
            tempVector.setY(tempVector.getY() + unitMomenta[column].getLinearPartY() * v.get(column));
            tempVector.setZ(tempVector.getZ() + unitMomenta[column].getLinearPartZ() * v.get(column));
            
            column++;
         }
         
         // Pack twist of body j w.r.t. elevator expressed in body j com
         twistCalculator.packRelativeTwist(bodyTwists[j], rootBody, rigidBodies[j]);
         // Change expressed in frame to center of mass frame
         bodyTwists[j].changeFrame(centerOfMassFrame);
         
         tempTwist.set(bodyTwists[j]);
         tempTwist.changeFrame(jointList[j].getSuccessor().getInertia().getExpressedInFrame());
         // Ad^{T} * I * J * v
         tempTwist.packMatrix(tempSpatialMotionMatrix, 0);
         CommonOps.mult(denseAdjTimesI[j], tempSpatialMotionMatrix, tempMatrix);
         tempSpatialForceVector.set(centerOfMassFrame,tempMatrix);
         bodyMomenta[j].set(tempSpatialForceVector);
      }
      
      tempVector.scale(1 / robotMass);
      
      comTwist.setLinearPart(tempVector);
      
      computeADotVLeftSide();
      computeADotVRightSide();
   }
   
   private void computeADotVLeftSide()
   {
      for(int i = 0; i < bodyTwists.length; i++)
      {
         tempTwist.set(comTwist);
         tempTwist.sub(bodyTwists[i]);
         
         // Left multiply by ad^{T} : this is separated out rather than creating matrix objects to save computation time.
         // See method below for creating adjoint from twist for reference.
         // ad^{T} * Ad^{T} * I * J * v
         // The order of these cross products are backwards because I need -a x b and cross products are anticommutative,
         // so -a x b = b x a.
         leftSide.setToZero();
         tempVector.cross(bodyMomenta[i].getLinearPart(), tempTwist.getLinearPart());
         leftSide.addAngularPart(tempVector);
         tempVector.cross(bodyMomenta[i].getAngularPart(), tempTwist.getAngularPart());
         leftSide.addAngularPart(tempVector);
         tempVector.cross(bodyMomenta[i].getLinearPart(), tempTwist.getAngularPart());
         leftSide.addLinearPart(tempVector);
         //
         leftSide.packMatrix(tempSpatialForceMatrix);
         CommonOps.add(aDotV, tempSpatialForceMatrix, aDotV);
      }
   }
   
   private void computeADotVRightSide()
   {
      // Here we calculate Jdot * v by computing the spatial acceleration with vddot = 0
      for(int j = 0; j<jointList.length; j++)
      {
         spatialAccelerationCalculator.packAccelerationOfBody(tempSpatialAcceleration, rigidBodies[j]);
         tempSpatialAcceleration.packMatrix(tempSpatialMotionMatrix, 0);
         CommonOps.mult(denseAdjTimesI[j], tempSpatialMotionMatrix, tempMatrix);
         CommonOps.add(aDotV, tempMatrix, aDotV);
      }
   }
   
   /**
    * Precomputes Ad^{T} * I since it is used twice in Adot*v and once in computing A.
    */
   private void precomputeAdjointTimesInertia()
   {
      tempAdjoint.zero();
      for(int i = 0; i<jointList.length; i++)
      {
         rigidBodies[i].getInertia().getExpressedInFrame().getTransformToDesiredFrame(tempTransform, centerOfMassFrame);
         tempTransform.get(tempMatrix3d,tempVector);
         
         set3By3MatrixBlock(tempAdjoint, 0, 0, tempMatrix3d);
         set3By3MatrixBlock(tempAdjoint, 3, 3, tempMatrix3d);
         MatrixTools.toTildeForm(tempPTilde, tempVector);
         tempPTilde.mul(tempMatrix3d);
         set3By3MatrixBlock(tempAdjoint, 0, 3, tempPTilde);
         
         rigidBodies[i].getInertia().packMatrix(tempInertiaMatrix);
         CommonOps.mult(tempAdjoint, tempInertiaMatrix, denseAdjTimesI[i]);
      }
   }
   
   private void set3By3MatrixBlock(DenseMatrix64F denseMatrix, int startRow, int startCol,Matrix3d matrix3d)
   {
      if(denseMatrix.numRows < 3 || denseMatrix.numCols < 3)
      {
         throw new RuntimeException("Your matrix must be 3x3 or larger.");
      }
      
      for(int i = 0; i < 3; i++)
      {
         for(int j = 0; j < 3; j++)
         {
            denseMatrix.unsafe_set(startRow+i, startCol+j, matrix3d.getElement(i, j));
         }
      }
   }

   public DenseMatrix64F getCentroidalMomentumMatrix()
   {
      return centroidalMomentumMatrix;
   }
   
   public DenseMatrix64F getADotVTerm()
   {
      return aDotV;
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      return centerOfMassFrame;
   }
}
