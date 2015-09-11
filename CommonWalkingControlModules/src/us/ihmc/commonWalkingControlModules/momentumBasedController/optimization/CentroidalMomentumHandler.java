package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TIntArrayList;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.MatrixYoVariableConversionTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;


/**
 * @author twan
 *         Date: 5/1/13
 */
public class CentroidalMomentumHandler
{
   private final boolean USE_NUMERICALLY_DIFFERENTIATED_CENTROIDAL_MOMENTUM_MATRIX = false;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final DenseMatrix64F adotV = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumMatrixPart = new DenseMatrix64F(1, 1);
   private final SpatialForceVector centroidalMomentumRate;

   private final DenseMatrix64F centroidalMomentumMatrixDerivative;
   private final DenseMatrix64F previousCentroidalMomentumMatrix;
   private final DoubleYoVariable[][] yoPreviousCentroidalMomentumMatrix;    // to make numerical differentiation rewindable

   private final double controlDT;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final DenseMatrix64F v;
   private final Map<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();
   private final DenseMatrix64F hdot = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumEquationRightHandSide = new DenseMatrix64F(Momentum.SIZE, 1);
   private final ReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateTermCalculator centroidalMomentumRateTermCalculator;

   public CentroidalMomentumHandler(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());

      this.centroidalMomentumMatrix = new CentroidalMomentumMatrix(ScrewTools.getRootBody(rootJoint.getPredecessor()), centerOfMassFrame);
      this.previousCentroidalMomentumMatrix = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());
      this.centroidalMomentumMatrixDerivative = new DenseMatrix64F(centroidalMomentumMatrix.getMatrix().getNumRows(),
              centroidalMomentumMatrix.getMatrix().getNumCols());
      yoPreviousCentroidalMomentumMatrix = new DoubleYoVariable[previousCentroidalMomentumMatrix.getNumRows()][previousCentroidalMomentumMatrix.getNumCols()];
      MatrixYoVariableConversionTools.populateYoVariables(yoPreviousCentroidalMomentumMatrix, "previousCMMatrix", registry);

      this.controlDT = controlDT;

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

      parentRegistry.addChild(registry);
      
      this.centroidalMomentumRateTermCalculator = new CentroidalMomentumRateTermCalculator(rootJoint.getPredecessor(),
            centerOfMassFrame, v, TotalMassCalculator.computeSubTreeMass(rootJoint.getSuccessor()));
   }

   public void initialize()
   {
      if(USE_NUMERICALLY_DIFFERENTIATED_CENTROIDAL_MOMENTUM_MATRIX)
      {
         centroidalMomentumMatrix.compute();
         previousCentroidalMomentumMatrix.set(centroidalMomentumMatrix.getMatrix());
         MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
      }
   }

   public void compute()
   {
      ScrewTools.packJointVelocitiesMatrix(jointsInOrder, v);
      
      if(USE_NUMERICALLY_DIFFERENTIATED_CENTROIDAL_MOMENTUM_MATRIX)
      {
         centroidalMomentumMatrix.compute();
         
         MatrixYoVariableConversionTools.getFromYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
         MatrixTools.numericallyDifferentiate(centroidalMomentumMatrixDerivative, previousCentroidalMomentumMatrix, centroidalMomentumMatrix.getMatrix(),
                 controlDT);
         MatrixYoVariableConversionTools.storeInYoVariables(previousCentroidalMomentumMatrix, yoPreviousCentroidalMomentumMatrix);
   
         CommonOps.mult(centroidalMomentumMatrixDerivative, v, adotV);
      }
      else
      {
         centroidalMomentumRateTermCalculator.compute();
         adotV.set(centroidalMomentumRateTermCalculator.getADotVTerm());
      }
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
         if(USE_NUMERICALLY_DIFFERENTIATED_CENTROIDAL_MOMENTUM_MATRIX)
         {
            MatrixTools.extractColumns(centroidalMomentumMatrix.getMatrix(), columnsForJoint, centroidalMomentumMatrixPart, startColumn);
         }
         else
         {
            MatrixTools.extractColumns(centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix(),
                  columnsForJoint, centroidalMomentumMatrixPart, startColumn);
         }
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

   public DenseMatrix64F getMomentumDotEquationRightHandSide(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();

      CommonOps.mult(momentumSubspace, momentumMultipliers, centroidalMomentumEquationRightHandSide);
      CommonOps.subtractEquals(centroidalMomentumEquationRightHandSide, adotV);
      return centroidalMomentumEquationRightHandSide;
   }
}
