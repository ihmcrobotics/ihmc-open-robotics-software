package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;

/**
 * @author twan Date: 5/1/13
 */
public class CentroidalMomentumHandler
{
   private final DenseMatrix64F adotV = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumMatrixPart = new DenseMatrix64F(1, 1);
   private final SpatialForceVector centroidalMomentumRate;

   private final InverseDynamicsJoint[] jointsInOrder;
   private final DenseMatrix64F v;
   private final Map<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();
   private final DenseMatrix64F momentum = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F momentumRate = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumEquationRightHandSide = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final ReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateTermCalculator centroidalMomentumRateTermCalculator;
   private final double robotMass;

   private boolean isUpToDate = false;

   public CentroidalMomentumHandler(RigidBody rootBody, ReferenceFrame centerOfMassFrame)
   {
      this.jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootBody);

      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      this.v = new DenseMatrix64F(nDegreesOfFreedom, 1);

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();
         columnsForJoints.put(joint, indices);
      }

      centroidalMomentumRate = new SpatialForceVector(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;

      robotMass = TotalMassCalculator.computeSubTreeMass(rootBody);
      this.centroidalMomentumRateTermCalculator = new CentroidalMomentumRateTermCalculator(rootBody, centerOfMassFrame, v, robotMass);
   }

   public void initialize()
   {
   }

   /**
    * Invalidates the internal data such that it will be updated when accessing any information such as the momentum or the centroidal momentum matrix.
    * @deprecated Use {@link #reset()} instead
    */
   public void compute()
   {
      reset();
   }

   /**
    * Invalidates the internal data such that it will be updated when accessing any information such as the momentum or the centroidal momentum matrix.
    */
   public void reset()
   {
      isUpToDate = false;
   }

   private void updateIfNeeded()
   {
      if (isUpToDate)
         return;

      isUpToDate = true;

      ScrewTools.getJointVelocitiesMatrix(jointsInOrder, v);

      centroidalMomentumRateTermCalculator.compute();
      adotV.set(centroidalMomentumRateTermCalculator.getADotVTerm());
   }

   public void getAngularMomentum(FrameVector3D angularMomentumToPack)
   {
      updateIfNeeded();
      DenseMatrix64F centroidalMomentumMatrix = centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix();
      CommonOps.mult(centroidalMomentumMatrix, v, momentum);
      angularMomentumToPack.setIncludingFrame(centerOfMassFrame, 0, momentum);
   }

   public void getLinearMomentum(FrameVector3D linearMomentumToPack)
   {
      updateIfNeeded();
      DenseMatrix64F centroidalMomentumMatrix = centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix();
      CommonOps.mult(centroidalMomentumMatrix, v, momentum);
      linearMomentumToPack.setIncludingFrame(centerOfMassFrame, 3, momentum);
   }

   public void getCenterOfMassVelocity(FrameVector3D centerOfMassVelocityToPack)
   {
      getLinearMomentum(centerOfMassVelocityToPack);
      centerOfMassVelocityToPack.scale(1.0 / robotMass);
   }

   public DenseMatrix64F getCentroidalMomentumMatrixPart(InverseDynamicsJoint[] joints)
   {
      updateIfNeeded();
      int partDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(joints);
      centroidalMomentumMatrixPart.reshape(Momentum.SIZE, partDegreesOfFreedom);
      centroidalMomentumMatrixPart.zero();
      int startColumn = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int[] columnsForJoint = columnsForJoints.get(joint);
         MatrixTools.extractColumns(centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix(), columnsForJoint, centroidalMomentumMatrixPart,
                                    startColumn);
         startColumn += columnsForJoint.length;
      }
      return centroidalMomentumMatrixPart;
   }

   public DenseMatrix64F getCentroidalMomentumConvectiveTerm()
   {
      updateIfNeeded();
      return adotV;
   }

   public void computeCentroidalMomentumRate(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations)
   {
      DenseMatrix64F centroidalMomentumMatrixPart = getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      CommonOps.mult(centroidalMomentumMatrixPart, jointAccelerations, momentumRate);
      CommonOps.addEquals(momentumRate, adotV);
      centroidalMomentumRate.set(centerOfMassFrame, momentumRate);
   }

   public SpatialForceVector getCentroidalMomentumRate()
   {
      updateIfNeeded();
      return centroidalMomentumRate;
   }

   public DenseMatrix64F getMomentumDotEquationRightHandSide(MomentumRateCommand momentumRateCommand)
   {
      updateIfNeeded();
      momentumRateCommand.getSelectionMatrix(centerOfMassFrame, selectionMatrix);
      DenseMatrix64F momentumRate = momentumRateCommand.getMomentumRate();

      CommonOps.mult(selectionMatrix, momentumRate, centroidalMomentumEquationRightHandSide);
      CommonOps.subtractEquals(centroidalMomentumEquationRightHandSide, adotV);
      return centroidalMomentumEquationRightHandSide;
   }
}
