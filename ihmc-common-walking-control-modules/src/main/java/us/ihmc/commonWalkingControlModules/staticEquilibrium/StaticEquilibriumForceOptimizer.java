package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

/**
 * This class solves for forces given a set of contact points, surface normals, and CoM xy position
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 */
public class StaticEquilibriumForceOptimizer
{
   private static final int maximumNumberOfIterations = 300;
   private static final double convergenceThreshold = 1e-6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final JavaQuadProgSolver qpSolver = new JavaQuadProgSolver();
   private final List<StaticEquilibriumContactPoint> contactPoints = new ArrayList<>();

   private int numberOfDecisionVariables;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj solution = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj lowerBounds = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj upperBounds = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Ain = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0, 0);

   private boolean feasibleSolutionFound;

   public StaticEquilibriumForceOptimizer()
   {
      for (int i = 0; i < StaticEquilibriumSolverInput.maxContactPoints; i++)
      {
         contactPoints.add(new StaticEquilibriumContactPoint(i, registry, graphicsListRegistry));
      }
   }

   public boolean solve(StaticEquilibriumSolverInput input, Point2DReadOnly centerOfMassXY)
   {
      if (!input.checkInput())
      {
         return false;
      }

      numberOfDecisionVariables = 4 * input.getNumberOfContacts();

      Aeq.reshape(6, numberOfDecisionVariables);
      beq.reshape(6, 1);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).clear();
      }

      for (int i = 0; i < input.getNumberOfContacts(); i++)
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

      beq.set(2, 0, input.getRobotMass() * input.getGravityMagnitude());
      beq.set(3, 0, input.getRobotMass() * input.getGravityMagnitude() * centerOfMassXY.getY());
      beq.set(4, 0, - input.getRobotMass() * input.getGravityMagnitude() * centerOfMassXY.getX());

      quadraticCost.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      CommonOps_DDRM.setIdentity(quadraticCost);
      linearCost.reshape(numberOfDecisionVariables, 1);

      lowerBounds.reshape(numberOfDecisionVariables, 1);
      upperBounds.reshape(numberOfDecisionVariables, 1);

      Ain.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      CommonOps_DDRM.setIdentity(Ain);
      CommonOps_DDRM.scale(-1.0, Ain);

      bin.reshape(numberOfDecisionVariables, 1);

      CommonOps_DDRM.fill(lowerBounds, 0.0);
      CommonOps_DDRM.fill(upperBounds, 1000.0);

      qpSolver.clear();
      qpSolver.resetActiveSet();
      qpSolver.setUseWarmStart(false);

      qpSolver.setMaxNumberOfIterations(maximumNumberOfIterations);
      qpSolver.setConvergenceThreshold(convergenceThreshold);

      qpSolver.setQuadraticCostFunction(quadraticCost, linearCost);
      qpSolver.setLinearEqualityConstraints(Aeq, beq);
      qpSolver.setVariableBounds(lowerBounds, upperBounds);
      qpSolver.setLinearInequalityConstraints(Ain, bin);

      try
      {
         qpSolver.solve(solution);
      }
      catch (Exception e)
      {
         feasibleSolutionFound = false;
         return false;
      }

      feasibleSolutionFound = true;
      for (int i = 0; i < numberOfDecisionVariables; i++)
      {
         if (Double.isNaN(solution.get(i)))
         {
            feasibleSolutionFound = false;
            break;
         }
      }

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         contactPoints.get(i).setResolvedForce(solution);
      }

      return feasibleSolutionFound;
   }

   public boolean feasibleSolutionFound()
   {
      return feasibleSolutionFound;
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
