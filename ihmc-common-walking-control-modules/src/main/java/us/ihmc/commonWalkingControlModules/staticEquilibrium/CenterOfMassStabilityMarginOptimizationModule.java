package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is an implementation of "Testing Static Equilibrium for Legged Robots", Bretl et al, 2008
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 *
 * Solves the LP:
 *
 * max_{x,f} c dot x                   (max com displacement)
 *    s.t.  mg + sum(f) = 0            (lin static equilibrium)
 *    s.t.  sum (x x f + x x mg) = 0   (ang static equilibrium)
 *          f is friction constrained
 *          tau_min <= G - J^T f <= tau_max  (f is actuation constrained)
 *
 * x is com position, c is a direction to optimize
 * Notation taken from EoM:
 * M qdd + C qd + G = tau + J^T f
 */
public class CenterOfMassStabilityMarginOptimizationModule
{
   private static final double GRAVITY = 9.81;
   private static final int NUM_BASIS_VECTORS = 4;
   private static final double COEFFICIENT_OF_FRICTION = 0.7;
   private static final double BASIS_VEC_ANGLE = Math.atan(COEFFICIENT_OF_FRICTION);
   static final int MAX_CONTACT_POINTS = 12;

   private static final int CoM_DIMENSIONS = 2;
   private static final int LINEAR_DIMENSIONS = 3;
   private static final int STATIC_EQUILIBRIUM_CONSTRAINTS = 6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final LinearProgramSolver linearProgramSolver = new LinearProgramSolver();
   private final double mg;

   private final List<YoFramePoint3D> contactPointPositions = new ArrayList<>();
   private final List<YoFrameVector3D> basisVectors = new ArrayList<>();

   private int numberOfContactPoints;
   /* Number of nominal decision variables: 3 * n_contact_points + 2 CoM directions */
   private int nominalDecisionVariables;
   /* Number of non-negative decision variables: n_basis_vectors + 2 * 2 CoM directions */
   private int posDecisionVariables;

   /* Equality matrices for A x = b, where x = [f_0x, f_0y, ..., c_x, c_y] */
   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);

   /* Conversion from nominal x, which is x = [f_0x, f_0y, ..., c_x, c_y] to positive x+, which is x+ = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] , and x+ > =0 */
   private final DMatrixRMaj posXToNominalX = new DMatrixRMaj(0);

   /* Inequality matrices for Ain x <= bin, where x = [f_0x, f_0y, ..., c_x, c_y] */
   private final DMatrixRMaj Ain = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);

   /* Inequality matrices for Ain_pos x+ <= bin where x+ = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   private final DMatrixRMaj Ain_pos = new DMatrixRMaj(0);

   /* Reward vector, based on query direction */
   private final DMatrixRMaj rewardVectorC = new DMatrixRMaj(0);
   private final DMatrixRMaj solution = new DMatrixRMaj(0);

   /* Whether LP solver converged or not */
   private boolean foundSolution = false;

   /* Position of optimized CoM */
   private final Point2D optimizedCoM = new Point2D();
   /* Yo-Position of optimized CoM */
   private final YoFramePoint3D yoOptimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);

   /* Indices for CoM position variables in x+ */
   private int cx_pos_index;
   private int cy_pos_index;
   private int cx_neg_index;
   private int cy_neg_index;

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final AxisAngle tempAxisAngle = new AxisAngle();

   public CenterOfMassStabilityMarginOptimizationModule(double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      mg = robotMass * GRAVITY;

      for (int i = 0; i < MAX_CONTACT_POINTS; i++)
      {
         contactPointPositions.add(new YoFramePoint3D("contactPoint" + i, ReferenceFrame.getWorldFrame(), registry));

         for (int j = 0; j < NUM_BASIS_VECTORS; j++)
         {
            basisVectors.add(new YoFrameVector3D("beta_" + i + "_" + j, ReferenceFrame.getWorldFrame(), registry));
         }
      }

      if (parentRegistry != null)
      {
         YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());
         for (int i = 0; i < MAX_CONTACT_POINTS; i++)
         {
            YoGraphicPosition contactPointGraphic = new YoGraphicPosition("contactPointGraphic" + i, contactPointPositions.get(i), 0.03, YoAppearance.Black());
            graphicsList.add(contactPointGraphic);

            for (int j = 0; j < NUM_BASIS_VECTORS; j++)
            {
               YoGraphicVector basisVectorGraphic = new YoGraphicVector("basisGraphic" + i + "_" + j, contactPointPositions.get(i), basisVectors.get(j), 0.15, YoAppearance.Black());
               graphicsList.add(basisVectorGraphic);
            }
         }
         graphicsList.add(new YoGraphicPosition("optimizedCoMGraphic", yoOptimizedCoM, 0.03, YoAppearance.Red()));

         graphicsListRegistry.registerYoGraphicsList(graphicsList);

         parentRegistry.addChild(registry);
      }
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      clear();

      /* Compute contact point positions and corresponding basis vectors */

      numberOfContactPoints = contactState.getNumberOfContactPoints();

      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         ReferenceFrame contactFrame = contactState.getContactFrame(contactIdx);

         tempPoint.setToZero(contactFrame);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         contactPointPositions.get(contactIdx).set(tempPoint);

         for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
         {
            tempVector.setIncludingFrame(contactFrame, Axis3D.Z);
            double axisPolarCoordinate = basisIdx * 2.0 * Math.PI / NUM_BASIS_VECTORS;
            tempAxisAngle.set(Math.cos(axisPolarCoordinate), Math.sin(axisPolarCoordinate), 0.0, BASIS_VEC_ANGLE);
            tempAxisAngle.transform(tempVector);

            tempVector.changeFrame(ReferenceFrame.getWorldFrame());
            basisVectors.get(getBasisIndex(contactIdx, basisIdx)).set(tempVector);
         }
      }

      /* Compute nominal equality constraint to enforce static equilibrium */

      nominalDecisionVariables = LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints() + CoM_DIMENSIONS;
      Aeq.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, nominalDecisionVariables);
      beq.reshape(STATIC_EQUILIBRIUM_CONSTRAINTS, 1);

      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         YoFramePoint3D contactPoint = contactPointPositions.get(contactIdx);

         int colOffset = 3 * contactIdx;

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

      int cx_index = nominalDecisionVariables - 2;
      int cy_index = nominalDecisionVariables - 1;

      Aeq.set(3, cy_index, -mg);
      Aeq.set(4, cx_index, mg);
      beq.set(2, 0, mg);

      /* Compute map from positive x to nominal x */

      posDecisionVariables = NUM_BASIS_VECTORS * contactState.getNumberOfContactPoints() + 2 * CoM_DIMENSIONS;
      posXToNominalX.reshape(nominalDecisionVariables, posDecisionVariables);

      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
         {
            int rowOffset = LINEAR_DIMENSIONS * contactIdx;
            int column = getBasisIndex(contactIdx, basisIdx);
            YoFrameVector3D basisVector = basisVectors.get(column);

            posXToNominalX.set(rowOffset + Axis3D.X.ordinal(), column, basisVector.getX());
            posXToNominalX.set(rowOffset + Axis3D.Y.ordinal(), column, basisVector.getY());
            posXToNominalX.set(rowOffset + Axis3D.Z.ordinal(), column, basisVector.getZ());
         }
      }

      cx_pos_index = posDecisionVariables - 4;
      cy_pos_index = posDecisionVariables - 3;
      cx_neg_index = posDecisionVariables - 2;
      cy_neg_index = posDecisionVariables - 1;

      posXToNominalX.set(cx_index, cx_pos_index, 1.0);
      posXToNominalX.set(cy_index, cy_pos_index, 1.0);
      posXToNominalX.set(cx_index, cx_neg_index, -1.0);
      posXToNominalX.set(cy_index, cy_neg_index, -1.0);

      /* Assemble nominal augmented inequality constraint */

      DMatrixRMaj A_actuation = contactState.getActuationConstraintMatrix();
      DMatrixRMaj b_actuation = contactState.getActuationConstraintVector();

      Ain.reshape(2 * Aeq.getNumRows() + A_actuation.getNumRows(), nominalDecisionVariables);
      bin.reshape(2 * beq.getNumRows() + b_actuation.getNumRows(), 1);

      MatrixTools.setMatrixBlock(Ain, 0, 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(Ain, Aeq.getNumRows(), 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(Ain, 2 * Aeq.getNumRows(), 0, A_actuation, 0, 0, A_actuation.getNumRows(), A_actuation.getNumCols(), 1.0);

      MatrixTools.setMatrixBlock(bin, 0, 0, beq, 0, 0, beq.getNumRows(), beq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, beq.getNumRows(), 0, beq, 0, 0, beq.getNumRows(), beq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(bin, 2 * beq.getNumRows(), 0, b_actuation, 0, 0, b_actuation.getNumRows(), b_actuation.getNumCols(), 1.0);

      /* Compute solver augmented inequality constraint */

      Ain_pos.reshape(Ain.getNumRows(), posDecisionVariables);
      CommonOps_DDRM.mult(Ain, posXToNominalX, Ain_pos);

      rewardVectorC.reshape(posDecisionVariables, 1);
   }

   private static int getBasisIndex(int contactIdx, int basisIdx)
   {
      return NUM_BASIS_VECTORS * contactIdx + basisIdx;
   }

   public boolean solve(double queryDirectionX, double queryDirectionY)
   {
      Arrays.fill(rewardVectorC.getData(), 0.0);

      rewardVectorC.set(cx_pos_index, 0, queryDirectionX);
      rewardVectorC.set(cx_neg_index, 0, -queryDirectionX);
      rewardVectorC.set(cy_pos_index, 0, queryDirectionY);
      rewardVectorC.set(cy_neg_index, 0, -queryDirectionY);

      foundSolution = linearProgramSolver.solve(rewardVectorC, Ain_pos, bin, solution);
      if (foundSolution)
      {
         optimizedCoM.set(rewardVectorC.get(cx_pos_index) - rewardVectorC.get(cx_neg_index), rewardVectorC.get(cy_pos_index) - rewardVectorC.get(cy_neg_index));
      }
      else
      {
         optimizedCoM.setToNaN();
      }

      yoOptimizedCoM.set(optimizedCoM, 0.0);

      return foundSolution;
   }

   public boolean foundSolution()
   {
      return foundSolution;
   }

   public Point2DReadOnly getOptimizedCoM()
   {
      return optimizedCoM;
   }

   public void getResolvedForce(int contactIdx, Vector3DBasics resolvedForceToPack)
   {
      resolvedForceToPack.setToZero();

      for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
      {
         int basisIndex = getBasisIndex(contactIdx, basisIdx);
         YoFrameVector3D basisVector = basisVectors.get(basisIndex);
         double rhoSolution = solution.get(basisIndex);

         resolvedForceToPack.addX(basisVector.getX() * rhoSolution);
         resolvedForceToPack.addY(basisVector.getY() * rhoSolution);
         resolvedForceToPack.addZ(basisVector.getZ() * rhoSolution);
      }
   }

   int getNumberOfContactPoints()
   {
      return numberOfContactPoints;
   }

   YoFramePoint3D getYoContactPointPosition(int contactIdx)
   {
      return contactPointPositions.get(contactIdx);
   }

   private void clear()
   {
      numberOfContactPoints = 0;
      nominalDecisionVariables = -1;
      posDecisionVariables = -1;
      Aeq.zero();
      beq.zero();
      Ain.zero();
      bin.zero();
      posXToNominalX.zero();
      Ain_pos.zero();
      rewardVectorC.zero();
      solution.zero();
      cx_pos_index = -1;
      cy_pos_index = -1;
      cx_neg_index = -1;
      cy_neg_index = -1;

      for (int i = 0; i < contactPointPositions.size(); i++)
      {
         contactPointPositions.get(i).setToNaN();
      }
      for (int i = 0; i < basisVectors.size(); i++)
      {
         basisVectors.get(i).setToNaN();
      }
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
