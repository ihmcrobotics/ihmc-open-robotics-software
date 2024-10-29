package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CenterOfMassStabilityMarginOptimizationModule extends StabilityMarginOptimizationModule
{
   static final int NUM_DYNAMICS_CONSTRAINTS = 6;
   static final int CoM_DIMENSIONS = 2;

   /* Indices for CoM position variables in x_force */
   private int cx_index;
   private int cy_index;

   /* Indices for CoM position variables in x_rho */
   private int cx_pos_index;
   private int cy_pos_index;
   private int cx_neg_index;
   private int cy_neg_index;

   public CenterOfMassStabilityMarginOptimizationModule(double robotMass)
   {
      this("", robotMass, null, null);
   }

   public CenterOfMassStabilityMarginOptimizationModule(String prefix, double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(prefix, robotMass, parentRegistry, graphicsListRegistry);
   }

   @Override
   int computeConstraintMatrices(WholeBodyContactStateInterface contactState, boolean contactPointsHaveChanged)
   {
      int nominalDecisionVariables = LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints() + CoM_DIMENSIONS;

      if (contactPointsHaveChanged)
      {
         Aeq.reshape(NUM_DYNAMICS_CONSTRAINTS, nominalDecisionVariables);
         beq.reshape(NUM_DYNAMICS_CONSTRAINTS, 1);

         for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
         {
            FramePoint3D contactPoint = contactPointPositions.get(contactIdx);
            int colOffset = LINEAR_DIMENSIONS * contactIdx;

            Aeq.set(0, colOffset + Axis3D.X.ordinal(), 1.0);
            Aeq.set(1, colOffset + Axis3D.Y.ordinal(), 1.0);
            Aeq.set(2, colOffset + Axis3D.Z.ordinal(), 1.0);

            Aeq.set(3, colOffset + Axis3D.Y.ordinal(), -contactPoint.getZ());
            Aeq.set(3, colOffset + Axis3D.Z.ordinal(), contactPoint.getY());
            Aeq.set(4, colOffset + Axis3D.X.ordinal(), contactPoint.getZ());
            Aeq.set(4, colOffset + Axis3D.Z.ordinal(), -contactPoint.getX());
            Aeq.set(5, colOffset + Axis3D.X.ordinal(), -contactPoint.getY());
            Aeq.set(5, colOffset + Axis3D.Y.ordinal(), contactPoint.getX());
         }

         cx_index = nominalDecisionVariables - 2;
         cy_index = nominalDecisionVariables - 1;

         Aeq.set(3, cy_index, -mg);
         Aeq.set(4, cx_index, mg);
         beq.set(2, 0, mg);
      }

      return nominalDecisionVariables;
   }

   @Override
   public int computeNumberOfSolverVariables(WholeBodyContactStateInterface contactState)
   {
      return NUM_BASIS_VECTORS * contactState.getNumberOfContactPoints() + 2 * CoM_DIMENSIONS;
   }

   @Override
   public void packAdditionalTransformationRows(DMatrixRMaj solverToNominalTransformation)
   {
      cx_pos_index = solverDecisionVariables - 4;
      cy_pos_index = solverDecisionVariables - 3;
      cx_neg_index = solverDecisionVariables - 2;
      cy_neg_index = solverDecisionVariables - 1;

      solverToNominalTransformation.set(cx_index, cx_pos_index, 1.0);
      solverToNominalTransformation.set(cy_index, cy_pos_index, 1.0);
      solverToNominalTransformation.set(cx_index, cx_neg_index, -1.0);
      solverToNominalTransformation.set(cy_index, cy_neg_index, -1.0);
   }

   @Override
   public void packRewardVectorC(DMatrixRMaj rewardVectorC, double queryDirectionX, double queryDirectionY)
   {
      rewardVectorC.set(cx_pos_index, 0, queryDirectionX);
      rewardVectorC.set(cx_neg_index, 0, -queryDirectionX);
      rewardVectorC.set(cy_pos_index, 0, queryDirectionY);
      rewardVectorC.set(cy_neg_index, 0, -queryDirectionY);
   }

   @Override
   void packOptimizedStabilityPoint(DMatrixRMaj solutionNominal, Point2D optimizedStabilityPoint)
   {
      optimizedStabilityPoint.set(solutionNominal.get(cx_index), solutionNominal.get(cy_index));
   }

   @Override
   void clear(boolean contactPointsHaveChanged)
   {
      if (contactPointsHaveChanged)
      {
         cx_index = -1;
         cy_index = -1;
         cx_pos_index = -1;
         cy_pos_index = -1;
         cx_neg_index = -1;
         cy_neg_index = -1;
      }
   }

   @Override
   ReferenceFrame getSolverFrame()
   {
      return ReferenceFrame.getWorldFrame();
   }

   @Override
   ColorDefinition getRegionGraphicColor()
   {
      return ColorDefinitions.Black();
   }

   @Override
   int getNumberOfNominalVariables()
   {
      return LINEAR_DIMENSIONS * numberOfContactPoints + CoM_DIMENSIONS;
   }

   @Override
   int getNumDynamicsConstraints()
   {
      return NUM_DYNAMICS_CONSTRAINTS;
   }

   @Override
   double getStabilityPointGradientCoefficient()
   {
      return 1.0;
   }
}
