package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.convexOptimization.linearProgram.SolverMethod;
import us.ihmc.convexOptimization.linearProgram.SolverStatistics.LinearProgramFailureReason;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.awt.*;
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
public abstract class StabilityMarginOptimizationModule implements SCS2YoGraphicHolder
{
   static final boolean DEBUG = true;
   static final double GRAVITY = 9.81;
   static final int NUM_BASIS_VECTORS = 4;
   static final int MAX_CONTACT_POINTS = 12;
   static final int LINEAR_DIMENSIONS = 3;

   private final String prefix;
   private final YoRegistry registry;
   private final YoRegistry debugRegistry;
   private final LinearProgramSolver linearProgramSolver = new LinearProgramSolver();
   protected final double mg;

   protected final List<FramePoint3D> contactPointPositions = new ArrayList<>();
   protected final List<FrameVector3D> basisVectors = new ArrayList<>();
   protected final List<YoFramePoint3D> yoContactPointPositions = new ArrayList<>();
   protected final List<YoFrameVector3D> yoBasisVectors = new ArrayList<>();
   protected final List<YoFrameVector3D> resolvedForces = new ArrayList<>();

   protected int numberOfContactPoints;
   /* Number of decision variables in x_nominal = [f_0x, f_0y, ..., c_x, c_y] */
   protected int nominalDecisionVariables;
   /* Number of decision variables in x_solver = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   protected int solverDecisionVariables;

   /* Equality matrices for Aeq x_nominal = beq, where x_nominal = [f_0x, f_0y, ..., c_x, c_y] are the nominal decision variables */
   final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   final DMatrixRMaj beq = new DMatrixRMaj(0);

   /* Transformation from x_solver = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] to x_nominal = [f_0x, f_0y, ..., c_x, c_y], with x_solver >= 0 */
   private final DMatrixRMaj solverToNominalTransformation = new DMatrixRMaj(0);

   /* Inequality matrices for Ain x_nominal <= bin, where x_nominal = [f_0x, f_0y, ..., c_x, c_y] */
   private final DMatrixRMaj Ain = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);

   /* Inequality matrices for Ain_solver x_solver <= bin where x_solver = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   private final DMatrixRMaj Ain_solver = new DMatrixRMaj(0);

   /* Reward vector, based on query direction */
   private final DMatrixRMaj rewardVectorC = new DMatrixRMaj(0);
   /* Solver solution for x_rho = [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] */
   private final DMatrixRMaj solutionSolver = new DMatrixRMaj(0);
   /* Solver solution for x_force = [f_0x, f_0y, ..., c_x, c_y] */
   private final DMatrixRMaj solutionNominal = new DMatrixRMaj(0);

   /* Whether LP solver converged or not */
   private boolean foundSolution = false;

   /* Position of optimized CoM */
   private final Point2D optimizedStabilityPoint = new Point2D();
   /* Yo-Position of optimized CoM */
   private final YoFramePoint3D yoOptimizedStabilityPoint;

   private final YoDouble equalityConstraintEpsilon;

   /* Yo-Variables to debug in cases the solver fails */
   private YoBoolean failedForPhaseI;
   private final YoEnum<LinearProgramFailureReason> failureReason;

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final AxisAngle tempAxisAngle = new AxisAngle();

   public StabilityMarginOptimizationModule(String prefix, double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.prefix = prefix;
      mg = robotMass * GRAVITY;
      registry = new YoRegistry(prefix + getClass().getSimpleName());
      debugRegistry = new YoRegistry(prefix + getClass().getSimpleName() + "Debug");

      for (int i = 0; i < MAX_CONTACT_POINTS; i++)
      {
         contactPointPositions.add(new FramePoint3D());
         yoContactPointPositions.add(new YoFramePoint3D("contactPoint" + i, ReferenceFrame.getWorldFrame(), debugRegistry));
         resolvedForces.add(new YoFrameVector3D("resolvedForce" + i, ReferenceFrame.getWorldFrame(), debugRegistry));

         for (int j = 0; j < NUM_BASIS_VECTORS; j++)
         {
            basisVectors.add(new FrameVector3D());
            yoBasisVectors.add(new YoFrameVector3D("beta_" + i + "_" + j, ReferenceFrame.getWorldFrame(), debugRegistry));
         }
      }

      yoOptimizedStabilityPoint = new YoFramePoint3D("optimizedStabilityPoint", ReferenceFrame.getWorldFrame(), registry);
      failedForPhaseI = new YoBoolean("failedForPhaseI", registry);
      failureReason = new YoEnum<>("failureReason", registry, LinearProgramFailureReason.class, true);

      if (DEBUG)
      {
         registry.addChild(debugRegistry);

         if (graphicsListRegistry != null)
         {
            YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());
            for (int contactIdx = 0; contactIdx < MAX_CONTACT_POINTS; contactIdx++)
            {
               YoGraphicPosition contactPointGraphic = new YoGraphicPosition(prefix + "contactPointGraphic" + contactIdx,
                                                                             yoContactPointPositions.get(contactIdx),
                                                                             0.01,
                                                                             YoAppearance.Black());
               graphicsList.add(contactPointGraphic);

               YoArtifactPosition contactPointArtifact = new YoArtifactPosition(prefix + "contactPointArtifact" + contactIdx,
                                                                                yoContactPointPositions.get(contactIdx).getYoX(),
                                                                                yoContactPointPositions.get(contactIdx).getYoY(),
                                                                                GraphicType.BALL,
                                                                                Color.BLACK,
                                                                                0.003);
               graphicsListRegistry.registerArtifact(getClass().getSimpleName(), contactPointArtifact);

               for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
               {
                  YoGraphicVector basisVectorGraphic = new YoGraphicVector(prefix + "basisGraphic" + contactIdx + "_" + basisIdx,
                                                                           yoContactPointPositions.get(contactIdx),
                                                                           yoBasisVectors.get(getBasisIndex(contactIdx, basisIdx)),
                                                                           0.15,
                                                                           YoAppearance.Black());
                  graphicsList.add(basisVectorGraphic);
               }
            }

            graphicsList.add(new YoGraphicPosition(prefix + "optimizedStabilityPointGraphic", yoOptimizedStabilityPoint, 0.03, YoAppearance.Red()));
            graphicsListRegistry.registerYoGraphicsList(graphicsList);
         }
      }

      equalityConstraintEpsilon = new YoDouble("equalityConstraintEpilson", registry);
      equalityConstraintEpsilon.set(0.0);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      updateContactState(contactState, true);
   }

   public void updateContactState(WholeBodyContactStateInterface contactState, boolean contactPointsHaveChanged)
   {
      clearInternal(contactPointsHaveChanged);

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////// Compute contact point positions and corresponding basis vectors ////////////////////////////////
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (contactPointsHaveChanged)
      {
         numberOfContactPoints = contactState.getNumberOfContactPoints();

         for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
         {
            ReferenceFrame contactFrame = contactState.getContactFrame(contactIdx);

            contactPointPositions.get(contactIdx).setToZero(contactFrame);
            contactPointPositions.get(contactIdx).changeFrame(getSolverFrame());
            yoContactPointPositions.get(contactIdx).setMatchingFrame(contactPointPositions.get(contactIdx));
            double basisVectorAngle = Math.atan(contactState.getCoefficientOfFriction(contactIdx));

            for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
            {
               FrameVector3D basisVector = basisVectors.get(getBasisIndex(contactIdx, basisIdx));
               basisVector.setIncludingFrame(contactFrame, Axis3D.Z);
               double axisPolarCoordinate = basisIdx * 2.0 * Math.PI / NUM_BASIS_VECTORS;
               tempAxisAngle.set(Math.cos(axisPolarCoordinate), Math.sin(axisPolarCoordinate), 0.0, basisVectorAngle);
               tempAxisAngle.transform(basisVector);

               basisVector.changeFrame(getSolverFrame());
               yoBasisVectors.get(getBasisIndex(contactIdx, basisIdx)).setMatchingFrame(basisVector);
            }
         }
      }

      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////// Compute nominal equality constraint to enforce static equilibrium ///////////////////////////////
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      nominalDecisionVariables = computeConstraintMatrices(contactState, contactPointsHaveChanged);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////////// Compute map from positive x to nominal x ///////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (contactPointsHaveChanged)
      {
         solverDecisionVariables = computeNumberOfSolverVariables(contactState);

         solverToNominalTransformation.reshape(nominalDecisionVariables, solverDecisionVariables);

         for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
         {
            for (int basisIdx = 0; basisIdx < NUM_BASIS_VECTORS; basisIdx++)
            {
               int rowOffset = LINEAR_DIMENSIONS * contactIdx;
               int column = getBasisIndex(contactIdx, basisIdx);
               YoFrameVector3D basisVector = yoBasisVectors.get(column);

               solverToNominalTransformation.set(rowOffset + Axis3D.X.ordinal(), column, basisVector.getX());
               solverToNominalTransformation.set(rowOffset + Axis3D.Y.ordinal(), column, basisVector.getY());
               solverToNominalTransformation.set(rowOffset + Axis3D.Z.ordinal(), column, basisVector.getZ());
            }
         }

         packAdditionalTransformationRows(solverToNominalTransformation);
      }

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /////////////////////////////////////// Assemble nominal augmented inequality constraint ///////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // Actuation constraint C f <= d, where f = [f_0x, f_0y, f_0z, f_1x... ] are the ground reaction forces in world frame
      DMatrixRMaj A_actuation = contactState.getActuationConstraintMatrix();
      DMatrixRMaj b_actuation = contactState.getActuationConstraintVector();

      // Ain [f c] <= bin  ---> [Aeq, -Aeq, A_actuation]^T [f c] <= [beq -beq b_actuation]^T
      Ain.reshape(2 * Aeq.getNumRows() + A_actuation.getNumRows(), nominalDecisionVariables);
      bin.reshape(2 * beq.getNumRows() + b_actuation.getNumRows(), 1);

      MatrixTools.setMatrixBlock(Ain, 0, 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(Ain, Aeq.getNumRows(), 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(Ain, 2 * Aeq.getNumRows(), 0, A_actuation, 0, 0, A_actuation.getNumRows(), A_actuation.getNumCols(), 1.0);

      MatrixTools.setMatrixBlock(bin, 0, 0, beq, 0, 0, beq.getNumRows(), beq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, beq.getNumRows(), 0, beq, 0, 0, beq.getNumRows(), beq.getNumCols(), -1.0);

      // Add equality constraint epsilon here so that it only effects the static equilibrium equality constraint
      CommonOps_DDRM.add(bin, equalityConstraintEpsilon.getValue());

      MatrixTools.setMatrixBlock(bin, 2 * beq.getNumRows(), 0, b_actuation, 0, 0, b_actuation.getNumRows(), b_actuation.getNumCols(), 1.0);

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////// Compute solver augmented inequality constraint ////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      Ain_solver.reshape(Ain.getNumRows(), solverDecisionVariables);
      CommonOps_DDRM.mult(Ain, solverToNominalTransformation, Ain_solver);

      rewardVectorC.reshape(solverDecisionVariables, 1);
   }

   abstract int computeConstraintMatrices(WholeBodyContactStateInterface contactState, boolean contactPointsHaveChanged);

   abstract int computeNumberOfSolverVariables(WholeBodyContactStateInterface contactState);

   abstract void packAdditionalTransformationRows(DMatrixRMaj solverToNominalTransformation);

   abstract void packRewardVectorC(DMatrixRMaj rewardVectorC, double queryDirectionX, double queryDirectionY);

   abstract void packOptimizedStabilityPoint(DMatrixRMaj solutionNominal, Point2D optimizedStabilityPoint);

   abstract void clear(boolean contactPointsHaveChanged);

   abstract ReferenceFrame getSolverFrame();

   abstract ColorDefinition getRegionGraphicColor();

   abstract int getNumberOfNominalVariables();

   abstract int getNumDynamicsConstraints();

   abstract double getStabilityPointGradientCoefficient();

   private static int getBasisIndex(int contactIdx, int basisIdx)
   {
      return NUM_BASIS_VECTORS * contactIdx + basisIdx;
   }

   public void setEqualityConstraintEpsilon(double epsilon)
   {
      this.equalityConstraintEpsilon.set(epsilon);
   }

   public boolean solve(double queryDirectionX, double queryDirectionY)
   {
      Arrays.fill(rewardVectorC.getData(), 0.0);
      packRewardVectorC(rewardVectorC, queryDirectionX, queryDirectionY);

      foundSolution = linearProgramSolver.solve(rewardVectorC, Ain_solver, bin, solutionSolver, SolverMethod.SIMPLEX);
      if (foundSolution)
      {
         CommonOps_DDRM.mult(solverToNominalTransformation, solutionSolver, solutionNominal);
         packOptimizedStabilityPoint(solutionNominal, optimizedStabilityPoint);
      }
      else
      {
         optimizedStabilityPoint.setToNaN();
      }

      failureReason.set(linearProgramSolver.getSimplexStatistics().getFailureReason());
      failedForPhaseI.set(linearProgramSolver.getSimplexStatistics().isFailedForPhaseI());

      if (foundSolution)
      {
         updateGraphics();
         yoOptimizedStabilityPoint.set(optimizedStabilityPoint, 0.0);
      }

      return foundSolution;
   }

   public Point2D solveForFixedBasis(TIntArrayList basisIndices)
   {
      linearProgramSolver.solveForFixedBasis(Ain_solver, bin, basisIndices, solutionSolver);
      CommonOps_DDRM.mult(solverToNominalTransformation, solutionSolver, solutionNominal);
      packOptimizedStabilityPoint(solutionNominal, optimizedStabilityPoint);
      return optimizedStabilityPoint;
   }

   private void updateGraphics()
   {
      if (!DEBUG)
         return;

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         getResolvedForce(i, resolvedForces.get(i));
      }
   }

   public LinearProgramSolver getLinearProgramSolver()
   {
      return linearProgramSolver;
   }

   public boolean foundSolution()
   {
      return foundSolution;
   }

   public Point2DReadOnly getOptimizedStabilityPoint()
   {
      return optimizedStabilityPoint;
   }

   public DMatrixRMaj getSolverSolution()
   {
      return solutionSolver;
   }

   public void getResolvedForce(int contactIdx, Vector3DBasics resolvedForceToPack)
   {
      resolvedForceToPack.setX(solutionNominal.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.X.ordinal()));
      resolvedForceToPack.setY(solutionNominal.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Y.ordinal()));
      resolvedForceToPack.setZ(solutionNominal.get(LINEAR_DIMENSIONS * contactIdx + Axis3D.Z.ordinal()));
   }

   /**
    * Returns the optimized vector of forces and CoM (if running CoM-based optimizer), [f_0x f_0y f_0z f_1x ... f_nz c_x c_y]
    */
   public DMatrixRMaj getNominalSolution()
   {
      return solutionNominal;
   }

   int getNumberOfContactPoints()
   {
      return numberOfContactPoints;
   }

   YoFramePoint3D getYoContactPointPosition(int contactIdx)
   {
      return yoContactPointPositions.get(contactIdx);
   }

   /**
    * Returns inequality constraint matrix Ain, for Ain x_force <= bin, where x_force = [f_0x, f_0y, ..., c_x, c_y]
    */
   public DMatrixRMaj getNominalConstraintMatrix()
   {
      return Ain;
   }

   /**
    * Returns transformation matrix from solver variables [rho_0, rho_1, ..., c_x+, c_y+, c_x-, c_y-] to nominal variables [f_0x, f_0y, ..., c_x, c_y]
    */
   public DMatrixRMaj getSolverToNominalTransformation()
   {
      return solverToNominalTransformation;
   }

   public DMatrixRMaj getSolverConstraintMatrix()
   {
      return Ain_solver;
   }

   public FrameVector3DReadOnly getBasisVector(int contactIdx, int basisIdx)
   {
      return basisVectors.get(getBasisIndex(contactIdx, basisIdx));
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(prefix + getClass().getSimpleName());

      if (DEBUG)
      {
         for (int i = 0; i < yoContactPointPositions.size(); i++)
         {
//            group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("contactPoint" + i,
//                                                                          yoContactPointPositions.get(i),
//                                                                          resolvedForces.get(i),
//                                                                          1.1 / mg,
//                                                                          ColorDefinitions.Red()));
            for (int j = 0; j < NUM_BASIS_VECTORS; j++)
            {
               group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("beta" + getBasisIndex(i, j),
                                                                             yoContactPointPositions.get(i),
                                                                             yoBasisVectors.get(getBasisIndex(i, j)),
                                                                             0.2,
                                                                             ColorDefinitions.Black()));
            }
         }

//         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("optimizedCoM", yoOptimizedCoM, 0.05, ColorDefinitions.Red()));
      }

      return group;
   }

   private void clearInternal(boolean contactPointsHaveChanged)
   {
      if (contactPointsHaveChanged)
      {
         numberOfContactPoints = 0;
         nominalDecisionVariables = -1;
         solverDecisionVariables = -1;
         Aeq.zero();
         beq.zero();
         solverToNominalTransformation.zero();

         for (int i = 0; i < yoContactPointPositions.size(); i++)
         {
            contactPointPositions.get(i).setToNaN();
            yoContactPointPositions.get(i).setToNaN();
         }
         for (int i = 0; i < yoBasisVectors.size(); i++)
         {
            basisVectors.get(i).setToNaN();
            yoBasisVectors.get(i).setToNaN();
         }
      }

      Ain.zero();
      bin.zero();
      Ain_solver.zero();
      rewardVectorC.zero();
      solutionSolver.zero();

      failedForPhaseI.set(false);
      failureReason.set(null);

      clear(contactPointsHaveChanged);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
