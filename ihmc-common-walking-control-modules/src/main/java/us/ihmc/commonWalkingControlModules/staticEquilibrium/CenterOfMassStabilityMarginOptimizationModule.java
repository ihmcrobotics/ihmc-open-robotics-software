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
 * This is an implementation of
 * <a href="https://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf">
 *    Testing Static Equilibrium for Legged Robots</a>.
 * and
 * <a href="https://arxiv.org/abs/1903.07999">
 *    Feasible Region: an Actuation-Aware Extension of the Support Region</a>.
 * <br>
 * The input is a query direction <tt>c</tt> and the output is a point <tt>x</tt> representing the maximum CoM displacement in the direction
 * of <tt>c</tt> given friction and actuation constraints.
 * <br>
 * Solves the LP:
 * <br>
 *
 * <pre>
 * max<sub>x,f</sub> c · x                           (max com displacement)
 *    s.t.  mg + &Sigma f = 0                  (lin static equilibrium)
 *          &Sigma x × f + x × mg = 0          (ang static equilibrium)
 *          f is friction constrained
 *          &tau<sub>min</sub> <= G - J^T f <= &tau<sub>max</sub>      (actuation constraints)
 * </pre>
 * Where:
 * <ul>
 *    <li>c is the query direction in R<sup>2</sup></li>
 *    <li>x is CoM position in R<sup>2</sup></li>
 * </ul>
 */
public class CenterOfMassStabilityMarginOptimizationModule
{
   private static final boolean DEBUG = false;
   static final double GRAVITY = 9.81;
   static final int NUM_BASIS_VECTORS = 4;
   static final int MAX_CONTACT_POINTS = 12;

   static final int CoM_DIMENSIONS = 2;
   static final int LINEAR_DIMENSIONS = 3;
   static final int STATIC_EQUILIBRIUM_CONSTRAINTS = 6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoRegistry debugRegistry = new YoRegistry(getClass().getSimpleName() + "Debug");
   private final LinearProgramSolver linearProgramSolver = new LinearProgramSolver();
   private final double mg;

   private final List<YoFramePoint3D> contactPointPositions = new ArrayList<>();
   private final List<YoFrameVector3D> basisVectors = new ArrayList<>();

   private int numberOfContactPoints;
   /* Number of decision variables in x_force = [f_0x, f_0y, ..., c_x, c_y] */
   private int nominalDecisionVariables;
   /* Number of decision variables in x_rho = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   private int rhoDecisionVariables;

   /* Equality matrices for Aeq x_force = beq, where x_force = [f_0x, f_0y, ..., c_x, c_y] are the force-based decision variables */
   final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   final DMatrixRMaj beq = new DMatrixRMaj(0);

   /* Conversion from x_force = [f_0x, f_0y, ..., c_x, c_y] to x_rho = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] , and x_rho > =0 */
   final DMatrixRMaj rhoToForce = new DMatrixRMaj(0);

   /* Inequality matrices for Ain x_force <= bin, where x_force = [f_0x, f_0y, ..., c_x, c_y] */
   private final DMatrixRMaj Ain = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);

   /* Inequality matrices for Ain_rho x_rho <= bin where x_rho = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   private final DMatrixRMaj Ain_rho = new DMatrixRMaj(0);

   /* Reward vector, based on query direction */
   private final DMatrixRMaj rewardVectorC = new DMatrixRMaj(0);
   /* Solver solution for x_rho = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   private final DMatrixRMaj solutionRho = new DMatrixRMaj(0);
   /* Solver solution for x_force = [f_0x, f_0y, ..., c_x, c_y] */
   private final DMatrixRMaj solutionForce = new DMatrixRMaj(0);

   /* Whether LP solver converged or not */
   private boolean foundSolution = false;

   /* Position of optimized CoM */
   private final Point2D optimizedCoM = new Point2D();
   /* Yo-Position of optimized CoM */
   private final YoFramePoint3D yoOptimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);

   /* Indices for CoM position variables in x_force */
   private int cx_index;
   private int cy_index;

   /* Indices for CoM position variables in x_rho */
   private int cx_pos_index;
   private int cy_pos_index;
   private int cx_neg_index;
   private int cy_neg_index;

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final AxisAngle tempAxisAngle = new AxisAngle();

   public CenterOfMassStabilityMarginOptimizationModule(double robotMass)
   {
      this(robotMass, null, null);
   }

   public CenterOfMassStabilityMarginOptimizationModule(double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      mg = robotMass * GRAVITY;

      for (int i = 0; i < MAX_CONTACT_POINTS; i++)
      {
         contactPointPositions.add(new YoFramePoint3D("contactPoint" + i, ReferenceFrame.getWorldFrame(), debugRegistry));

         for (int j = 0; j < NUM_BASIS_VECTORS; j++)
         {
            basisVectors.add(new YoFrameVector3D("beta_" + i + "_" + j, ReferenceFrame.getWorldFrame(), debugRegistry));
         }
      }

      if (DEBUG)
         registry.addChild(debugRegistry);

      if (DEBUG && graphicsListRegistry != null)
      {
         YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());
         for (int contactIdx = 0; contactIdx < MAX_CONTACT_POINTS; contactIdx++)
         {
            YoGraphicPosition contactPointGraphic = new YoGraphicPosition("contactPointGraphic" + contactIdx, contactPointPositions.get(contactIdx), 0.01, YoAppearance.Black());
            graphicsList.add(contactPointGraphic);

            for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
            {
               YoGraphicVector basisVectorGraphic = new YoGraphicVector("basisGraphic" + contactIdx + "_" + basisIdx, contactPointPositions.get(contactIdx), basisVectors.get(getBasisIndex(contactIdx, basisIdx)), 0.15, YoAppearance.Black());
               graphicsList.add(basisVectorGraphic);
            }
         }

         graphicsList.add(new YoGraphicPosition("optimizedCoMGraphic", yoOptimizedCoM, 0.03, YoAppearance.Red()));
         graphicsListRegistry.registerYoGraphicsList(graphicsList);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      clear();

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////// Compute contact point positions and corresponding basis vectors ////////////////////////////////
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      numberOfContactPoints = contactState.getNumberOfContactPoints();

      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         ReferenceFrame contactFrame = contactState.getContactFrame(contactIdx);

         tempPoint.setToZero(contactFrame);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         contactPointPositions.get(contactIdx).set(tempPoint);
         double basisVectorAngle = Math.atan(contactState.getCoefficientOfFriction(contactIdx));

         for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
         {
            tempVector.setIncludingFrame(contactFrame, Axis3D.Z);
            double axisPolarCoordinate = basisIdx * 2.0 * Math.PI / NUM_BASIS_VECTORS;
            tempAxisAngle.set(Math.cos(axisPolarCoordinate), Math.sin(axisPolarCoordinate), 0.0, basisVectorAngle);
            tempAxisAngle.transform(tempVector);

            tempVector.changeFrame(ReferenceFrame.getWorldFrame());
            basisVectors.get(getBasisIndex(contactIdx, basisIdx)).set(tempVector);
         }
      }

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////// Compute nominal equality constraint to enforce static equilibrium ///////////////////////////////
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

      cx_index = nominalDecisionVariables - 2;
      cy_index = nominalDecisionVariables - 1;

      Aeq.set(3, cy_index, -mg);
      Aeq.set(4, cx_index, mg);
      beq.set(2, 0, mg);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////// Compute map from positive x to nominal x ///////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      rhoDecisionVariables = NUM_BASIS_VECTORS * contactState.getNumberOfContactPoints() + 2 * CoM_DIMENSIONS;
      rhoToForce.reshape(nominalDecisionVariables, rhoDecisionVariables);

      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
         {
            int rowOffset = LINEAR_DIMENSIONS * contactIdx;
            int column = getBasisIndex(contactIdx, basisIdx);
            YoFrameVector3D basisVector = basisVectors.get(column);

            rhoToForce.set(rowOffset + Axis3D.X.ordinal(), column, basisVector.getX());
            rhoToForce.set(rowOffset + Axis3D.Y.ordinal(), column, basisVector.getY());
            rhoToForce.set(rowOffset + Axis3D.Z.ordinal(), column, basisVector.getZ());
         }
      }

      cx_pos_index = rhoDecisionVariables - 4;
      cy_pos_index = rhoDecisionVariables - 3;
      cx_neg_index = rhoDecisionVariables - 2;
      cy_neg_index = rhoDecisionVariables - 1;

      rhoToForce.set(cx_index, cx_pos_index, 1.0);
      rhoToForce.set(cy_index, cy_pos_index, 1.0);
      rhoToForce.set(cx_index, cx_neg_index, -1.0);
      rhoToForce.set(cy_index, cy_neg_index, -1.0);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////// Assemble nominal augmented inequality constraint ///////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Actuation constraint C f <= d, where f = [f_0x, f_0y, f_0z, f_1x... ] are teh ground reaction forces in world frame
      DMatrixRMaj A_actuation = contactState.getActuationConstraintMatrix();
      DMatrixRMaj b_actuation = contactState.getActuationConstraintVector();

      // Ain [f c] <= bin  ---> diag(Aeq, -Aeq, A_actuation) [f c] <= [beq -beq b_actuation]
      Ain.reshape(2 * Aeq.getNumRows() + A_actuation.getNumRows(), nominalDecisionVariables);
      bin.reshape(2 * beq.getNumRows() + b_actuation.getNumRows(), 1);

      MatrixTools.setMatrixBlock(Ain, 0, 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(Ain, Aeq.getNumRows(), 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(Ain, 2 * Aeq.getNumRows(), 0, A_actuation, 0, 0, A_actuation.getNumRows(), A_actuation.getNumCols(), 1.0);

      MatrixTools.setMatrixBlock(bin, 0, 0, beq, 0, 0, beq.getNumRows(), beq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, beq.getNumRows(), 0, beq, 0, 0, beq.getNumRows(), beq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(bin, 2 * beq.getNumRows(), 0, b_actuation, 0, 0, b_actuation.getNumRows(), b_actuation.getNumCols(), 1.0);


      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////// Compute solver augmented inequality constraint ////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      Ain_rho.reshape(Ain.getNumRows(), rhoDecisionVariables);
      CommonOps_DDRM.mult(Ain, rhoToForce, Ain_rho);

      rewardVectorC.reshape(rhoDecisionVariables, 1);
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

      foundSolution = linearProgramSolver.solve(rewardVectorC, Ain_rho, bin, solutionRho);
      if (foundSolution)
      {
         CommonOps_DDRM.mult(rhoToForce, solutionRho, solutionForce);
         optimizedCoM.set(solutionForce.get(cx_index), solutionForce.get(cy_index));
      }
      else
      {
         optimizedCoM.setToNaN();
      }

      yoOptimizedCoM.set(optimizedCoM, 0.0);

      return foundSolution;
   }

   public LinearProgramSolver getLinearProgramSolver()
   {
      return linearProgramSolver;
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
      resolvedForceToPack.setX(solutionForce.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.X.ordinal()));
      resolvedForceToPack.setY(solutionForce.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Y.ordinal()));
      resolvedForceToPack.setZ(solutionForce.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Z.ordinal()));
   }

   /**
    * Returns the optimized (3n_c + 2) x 1 vector of forces and CoM, [f_0x f_0y f_0z f_1x ... f_nz c_x c_y]
    */
   public DMatrixRMaj getOptimizedForceAndCoM()
   {
      return solutionForce;
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
      rhoDecisionVariables = -1;
      Aeq.zero();
      beq.zero();
      Ain.zero();
      bin.zero();
      rhoToForce.zero();
      Ain_rho.zero();
      rewardVectorC.zero();
      solutionRho.zero();

      cx_index = -1;
      cy_index = -1;
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
