package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
   private final DenseMatrix64F hdot = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumEquationRightHandSide = new DenseMatrix64F(Momentum.SIZE, 1);
   private final ReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateTermCalculator centroidalMomentumRateTermCalculator;

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

      double robotMass = TotalMassCalculator.computeSubTreeMass(rootBody);
      this.centroidalMomentumRateTermCalculator = new CentroidalMomentumRateTermCalculator(rootBody, centerOfMassFrame, v, robotMass);
   }

   public void initialize()
   {
   }

   public void compute()
   {
      ScrewTools.getJointVelocitiesMatrix(jointsInOrder, v);

      centroidalMomentumRateTermCalculator.compute();
      adotV.set(centroidalMomentumRateTermCalculator.getADotVTerm());
   }

   public DenseMatrix64F getCentroidalMomentumMatrixPart(InverseDynamicsJoint[] joints)
   {
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
      return adotV;
   }

   public void computeCentroidalMomentumRate(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations)
   {
      DenseMatrix64F centroidalMomentumMatrixPart = getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      CommonOps.mult(centroidalMomentumMatrixPart, jointAccelerations, hdot);
      CommonOps.addEquals(hdot, adotV);
      centroidalMomentumRate.set(centerOfMassFrame, hdot);
   }

   public SpatialForceVector getCentroidalMomentumRate()
   {
      return centroidalMomentumRate;
   }

   public DenseMatrix64F getMomentumDotEquationRightHandSide(MomentumRateCommand momentumRateCommand)
   {
      DenseMatrix64F selectionMatrix = momentumRateCommand.getSelectionMatrix();
      DenseMatrix64F momentumRate = momentumRateCommand.getMomentumRate();

      CommonOps.mult(selectionMatrix, momentumRate, centroidalMomentumEquationRightHandSide);
      CommonOps.subtractEquals(centroidalMomentumEquationRightHandSide, adotV);
      return centroidalMomentumEquationRightHandSide;
   }
}
