package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an implementation of "Testing Static Equilibrium for Legged Robots", Bretl et al, 2008
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 *
 * It solves for the (convex) region of feasible CoM XY positions, modelling the robot as a point mass
 * and imposing friction constraints.
 */
public class StaticEquilibriumSolver
{
   private static final int numberOfDirectionsToOptimize = 16;
   private static final double rhoMax = 1.0e4;
   private static final double convergenceThreshold = 1.0e-9;
   private static final int maximumNumberOfIterations = 500;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

//   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();
   private final JavaQuadProgSolver qpSolver = new JavaQuadProgSolver();

   private final List<StaticEquilibriumContactPoint> contactPoints = new ArrayList<>();

   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj lowerBounds = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj upperBounds = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj inequalityMatrix = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj solution = new DMatrixRMaj(0, 0);
   private final List<Point2D> supportRegion = new ArrayList<>();

   private final YoFramePoint3D directionToOptimizeBase = new YoFramePoint3D("directionToOptimizeBase", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D directionToOptimize = new YoFrameVector3D("directionToOptimize", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D nanCoM = new YoFramePoint3D("nanCoM", ReferenceFrame.getWorldFrame(), registry);

   private static final List<Vector2D> directionsToOptimize = new ArrayList<>();

   static
   {
      double dTheta = 2.0 * Math.PI / numberOfDirectionsToOptimize;
      for (int i = 0; i < numberOfDirectionsToOptimize; i++)
      {
         directionsToOptimize.add(new Vector2D(Math.cos(i * dTheta), Math.sin(i * dTheta)));
      }
   }

   public StaticEquilibriumSolver()
   {
      qpSolver.setConvergenceThreshold(convergenceThreshold);
      qpSolver.setMaxNumberOfIterations(maximumNumberOfIterations);

      for (int i = 0; i < StaticEquilibriumSolverInput.maxContactPoints; i++)
      {
         contactPoints.add(new StaticEquilibriumContactPoint(i, registry, graphicsListRegistry));
      }

      YoGraphicVector directionToOptimizeGraphic = new YoGraphicVector("directionToOptimizeGraphic", directionToOptimizeBase, directionToOptimize, 0.5);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), directionToOptimizeGraphic);
      directionToOptimizeBase.set(0.0, 0.0, 0.4);

      YoGraphicPosition optimizedCoMGraphic = new YoGraphicPosition("optimizedCoMGraphic", optimizedCoM, 0.05, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), optimizedCoMGraphic);

      YoGraphicPosition naNCoMGraphic = new YoGraphicPosition("naNCoMGraphic", nanCoM, 0.05, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), naNCoMGraphic);
   }

   public void solve(StaticEquilibriumSolverInput input)
   {
      if (!input.checkInput())
      {
         return;
      }

      int numberOfContactPoints = input.getNumberOfContacts();
      int numberOfDecisionVariables = 4 * numberOfContactPoints + 2;

      quadraticCost.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      linearCost.reshape(numberOfDecisionVariables, 1);

      CommonOps_DDRM.setIdentity(quadraticCost);
      CommonOps_DDRM.scale(1e-2, quadraticCost);
      CommonOps_DDRM.scale(10.0, linearCost);

      Aeq.reshape(6, numberOfDecisionVariables);
      beq.reshape(6, 1);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).clear();
      }

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         StaticEquilibriumContactPoint contactPoint = contactPoints.get(i);
         contactPoint.initialize(input);

         FramePoint3D contactPointPosition = input.getContactPointPositions().get(i);

         for (int j = 0; j < 4; j++)
         {
            YoFrameVector3D basisVector = contactPoint.getBasisVector(j);
            int column = 4 * i + j;

            Aeq.set(0, column, basisVector.getX());
            Aeq.set(1, column, basisVector.getY());
            Aeq.set(2, column, basisVector.getZ());

            // x-component of cross product
            double xMomentScale = contactPointPosition.getY() * basisVector.getZ() - contactPointPosition.getZ() * basisVector.getY();
            Aeq.set(3, column, xMomentScale);

            // y-component of cross product
            double yMomentScale = contactPointPosition.getZ() * basisVector.getX() - contactPointPosition.getX() * basisVector.getZ();
            Aeq.set(4, column, yMomentScale);

            // z-component of cross product
            double zMomentScale = contactPointPosition.getX() * basisVector.getY() - contactPointPosition.getY() * basisVector.getX();
            Aeq.set(5, column, zMomentScale);
         }
      }

      Aeq.set(3, numberOfDecisionVariables - 1, - input.getRobotMass() * input.getGravityMagnitude());
      Aeq.set(4, numberOfDecisionVariables - 2, input.getRobotMass() * input.getGravityMagnitude());
      beq.set(2, 0, input.getRobotMass() * input.getGravityMagnitude());

      lowerBounds.reshape(numberOfDecisionVariables, 1);
      upperBounds.reshape(numberOfDecisionVariables, 1);

      double comMinMax = 1.0e3;
      for (int i = 0; i < numberOfDecisionVariables; i++)
      {
         boolean isRhoConstraint = i < numberOfDecisionVariables - 2;
         lowerBounds.set(i, 0, isRhoConstraint ? 0.0 : - comMinMax);
         upperBounds.set(i, 0, isRhoConstraint ? rhoMax : comMinMax);
      }

      inequalityMatrix.reshape(numberOfDecisionVariables - 2, numberOfDecisionVariables);
      for (int i = 0; i < numberOfDecisionVariables - 2; i++)
      {
         inequalityMatrix.set(i, i, -1.0);
      }

      solution.reshape(numberOfDecisionVariables, 1);

      for (int i = 0; i < directionsToOptimize.size(); i++)
      {
         Vector2D directionToOptimize = directionsToOptimize.get(i);
         linearCost.set(numberOfDecisionVariables - 2, 0, - directionToOptimize.getX());
         linearCost.set(numberOfDecisionVariables - 1, 0, - directionToOptimize.getY());

         qpSolver.clear();
         qpSolver.resetActiveSet();
         qpSolver.setUseWarmStart(false);

         qpSolver.setQuadraticCostFunction(quadraticCost, linearCost);
         qpSolver.setLinearEqualityConstraints(Aeq, beq);
         qpSolver.setVariableBounds(lowerBounds, upperBounds);
         qpSolver.solve(solution);

         double comExtremumX = solution.get(numberOfDecisionVariables - 2, 0);
         double comExtremumY = solution.get(numberOfDecisionVariables - 1, 0);
         supportRegion.add(new Point2D(comExtremumX, comExtremumY));

         for (int j = 0; j < numberOfContactPoints; j++)
         {
            contactPoints.get(j).setResolvedForce(solution);
         }

         this.directionToOptimize.set(directionToOptimize);

         boolean containsNaN = containsNaN(solution);

         if (containsNaN)
         {
            this.optimizedCoM.setToNaN();
            this.nanCoM.set(0.0, 0.0, 0.1);
         }
         else
         {
            this.optimizedCoM.setX(solution.get(numberOfDecisionVariables - 2));
            this.optimizedCoM.setY(solution.get(numberOfDecisionVariables - 1));
            this.optimizedCoM.setZ(0.1);
            this.nanCoM.setToNaN();
         }

         if (tickAndUpdatable != null)
         {
            tickAndUpdatable.tickAndUpdate();
         }
      }
   }

   private static boolean containsNaN(DMatrixRMaj matrix)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
      {
         if (Double.isNaN(matrix.get(i)))
            return true;
      }

      return false;
   }

   public List<Point2D> getSupportRegion()
   {
      return supportRegion;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getGraphicsListRegistry()
   {
      return graphicsListRegistry;
   }

   public void setTickAndUpdatable(TickAndUpdatable tickAndUpdatable)
   {
      this.tickAndUpdatable = tickAndUpdatable;
   }
}
