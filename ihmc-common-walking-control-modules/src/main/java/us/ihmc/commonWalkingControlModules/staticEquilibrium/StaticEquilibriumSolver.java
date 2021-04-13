package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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
   }

   public void solve(StaticEquilibriumSolverInput input)
   {
      if (!input.checkInput())
      {
         return;
      }

      int numberOfContactPoints = input.getNumberOfContacts();
      int numberOfDecisionVariables = 12 * numberOfContactPoints + 2;

      quadraticCost.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      linearCost.reshape(numberOfDecisionVariables, 1);

      CommonOps_DDRM.setIdentity(quadraticCost);
      CommonOps_DDRM.scale(1e-5, quadraticCost);

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
         int contactPointStart = 12 * i;

         for (int j = 0; j < 4; j++)
         {
            YoFrameVector3D basisVector = contactPoint.getBasisVector(j);
            int columnStart = contactPointStart + 3 * j;

            Aeq.set(0, columnStart + 0, basisVector.getX());
            Aeq.set(1, columnStart + 1, basisVector.getY());
            Aeq.set(2, columnStart + 2, basisVector.getZ());

            // x-component of cross product
            Aeq.set(3, columnStart + 1, - contactPointPosition.getZ() * basisVector.getY());
            Aeq.set(3, columnStart + 2, contactPointPosition.getY() * basisVector.getZ());

            // y-component of cross product
            Aeq.set(4, columnStart + 0, contactPointPosition.getZ() * basisVector.getX());
            Aeq.set(4, columnStart + 2, - contactPointPosition.getX() * basisVector.getZ());

            // z-component of cross product
            Aeq.set(5, columnStart + 0, - contactPointPosition.getY() * basisVector.getX());
            Aeq.set(5, columnStart + 1, contactPointPosition.getX() * basisVector.getY());
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

         System.out.println("-------------------------");
         for (int j = 0; j < numberOfDecisionVariables; j++)
         {
            System.out.println(solution.get(j, 0));
         }
         System.out.println("-------------------------");

         double comExtremumX = solution.get(numberOfDecisionVariables - 2, 0);
         double comExtremumY = solution.get(numberOfDecisionVariables - 1, 0);
         supportRegion.add(new Point2D(comExtremumX, comExtremumY));
      }
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
}
