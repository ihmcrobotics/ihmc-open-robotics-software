package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CenterOfPressureStabilityMarginOptimizationModule extends StabilityMarginOptimizationModule
{
   static final int NUM_DYNAMICS_CONSTRAINTS = 4;

   /* Optimized net force */
   private final Vector3D optimizedLIPMForce = new Vector3D();

   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame midFeetZUpFrame;

   private final FramePoint3D midFootPoint = new FramePoint3D();
   private final FramePoint3D comPoint = new FramePoint3D();
   private double forceToCoPOffsetMultiplier;

   private final Vector2D optimizedLIPMForce2D = new Vector2D();
   private final Vector2D comToCop = new Vector2D();
   private final DMatrixRMaj rewardVectorCNominal = new DMatrixRMaj(0);

   public CenterOfPressureStabilityMarginOptimizationModule(String prefix,
                                                            double robotMass,
                                                            ReferenceFrame centerOfMassFrame,
                                                            ReferenceFrame midFeetZUpFrame,
                                                            YoRegistry parentRegistry,
                                                            YoGraphicsListRegistry graphicsListRegistry)
   {
      super(prefix, robotMass, parentRegistry, graphicsListRegistry);

      this.centerOfMassFrame = centerOfMassFrame;
      this.midFeetZUpFrame = midFeetZUpFrame;
   }

   @Override
   int computeConstraintMatrices(WholeBodyContactStateInterface contactState, boolean contactPointsHaveChanged)
   {
      int nominalDecisionVariables = LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints();
      Aeq.reshape(NUM_DYNAMICS_CONSTRAINTS, nominalDecisionVariables);
      beq.reshape(NUM_DYNAMICS_CONSTRAINTS, 1);

      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         FramePoint3D contactPoint = contactPointPositions.get(contactIdx);
         int colOffset = LINEAR_DIMENSIONS * contactIdx;

         Aeq.set(0, colOffset + Axis3D.Z.ordinal(), 1.0);

         Aeq.set(1, colOffset + Axis3D.Y.ordinal(), -contactPoint.getZ());
         Aeq.set(1, colOffset + Axis3D.Z.ordinal(), contactPoint.getY());
         Aeq.set(2, colOffset + Axis3D.X.ordinal(), contactPoint.getZ());
         Aeq.set(2, colOffset + Axis3D.Z.ordinal(), -contactPoint.getX());
         Aeq.set(3, colOffset + Axis3D.X.ordinal(), -contactPoint.getY());
         Aeq.set(3, colOffset + Axis3D.Y.ordinal(), contactPoint.getX());
      }

      beq.set(0, 0, mg);
      beq.set(1, 0, 0.0);
      beq.set(2, 0, 0.0);
      beq.set(3, 0, 0.0);

      return nominalDecisionVariables;
   }

   @Override
   int computeNumberOfSolverVariables(WholeBodyContactStateInterface contactState)
   {
      return NUM_BASIS_VECTORS * contactState.getNumberOfContactPoints();
   }

   @Override
   void packAdditionalTransformationRows(DMatrixRMaj solverToNominalTransformation)
   {
      // None extra
   }

   @Override
   void packRewardVectorC(DMatrixRMaj rewardVectorC, double queryDirectionX, double queryDirectionY)
   {
      rewardVectorCNominal.reshape(nominalDecisionVariables, 1);

      for (int contactIdx = 0; contactIdx < numberOfContactPoints; contactIdx++)
      {
         int colOffset = LINEAR_DIMENSIONS * contactIdx;
         rewardVectorCNominal.set(colOffset + Axis3D.X.ordinal(), 0, queryDirectionX);
         rewardVectorCNominal.set(colOffset + Axis3D.Y.ordinal(), 0, queryDirectionY);
      }

      CommonOps_DDRM.multTransA(rewardVectorCNominal, getSolverToNominalTransformation(), rewardVectorC);
      CommonOps_DDRM.transpose(rewardVectorC);

      comPoint.setToZero(centerOfMassFrame);
      midFootPoint.setToZero(midFeetZUpFrame);

      comPoint.changeFrame(ReferenceFrame.getWorldFrame());
      midFootPoint.changeFrame(ReferenceFrame.getWorldFrame());

      double dz = comPoint.getZ() - midFootPoint.getZ();
      forceToCoPOffsetMultiplier = -dz / mg;
   }

   @Override
   void packOptimizedStabilityPoint(DMatrixRMaj solutionNominal, Point2D optimizedStabilityPoint)
   {
      optimizedLIPMForce.setToZero();

      for (int contactIdx = 0; contactIdx < numberOfContactPoints; contactIdx++)
      {
         int colOffset = LINEAR_DIMENSIONS * contactIdx;
         optimizedLIPMForce.addX(solutionNominal.get(colOffset + Axis3D.X.ordinal(), 0));
         optimizedLIPMForce.addY(solutionNominal.get(colOffset + Axis3D.Y.ordinal(), 0));
         optimizedLIPMForce.addZ(solutionNominal.get(colOffset + Axis3D.Z.ordinal(), 0));
      }

      optimizedLIPMForce2D.set(optimizedLIPMForce);
      comToCop.setAndScale(forceToCoPOffsetMultiplier, optimizedLIPMForce2D);

      optimizedStabilityPoint.set(comPoint);
      optimizedStabilityPoint.add(comToCop);
   }

   @Override
   void clear(boolean contactPointsHaveChanged)
   {

   }

   @Override
   ReferenceFrame getSolverFrame()
   {
      return centerOfMassFrame;
   }

   @Override
   ColorDefinition getRegionGraphicColor()
   {
      return ColorDefinitions.Orange();
   }

   @Override
   int getNumberOfNominalVariables()
   {
      return LINEAR_DIMENSIONS * numberOfContactPoints;
   }

   @Override
   int getNumDynamicsConstraints()
   {
      return NUM_DYNAMICS_CONSTRAINTS;
   }

   @Override
   public double getStabilityPointGradientCoefficient()
   {
      return forceToCoPOffsetMultiplier;
   }

   public FramePoint3D getMidFootPoint()
   {
      return midFootPoint;
   }
}
