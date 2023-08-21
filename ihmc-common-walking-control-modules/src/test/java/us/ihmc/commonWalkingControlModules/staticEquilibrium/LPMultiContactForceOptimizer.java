package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.linear.*;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.ContactPoint.basisVectorsPerContactPoint;
import static us.ihmc.commonWalkingControlModules.staticEquilibrium.MultiContactFrictionBasedSupportRegionSolver.mg;

/**
 * This class solves for forces given a set of contact points, surface normals, and CoM xy position
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 */
public class LPMultiContactForceOptimizer
{
   private static final int maximumNumberOfIterations = 10000;
   private static final double convergenceThreshold = 1e-8;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final List<ContactPoint> contactPoints = new ArrayList<>();
   private final SimplexSolver solver = new SimplexSolver(convergenceThreshold);

   private int numberOfDecisionVariables;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);
   private double[] solution;

   public LPMultiContactForceOptimizer()
   {
      for (int i = 0; i < MultiContactFrictionBasedSupportRegionSolverInput.maxContactPoints; i++)
      {
         contactPoints.add(new ContactPoint(i, registry, graphicsListRegistry));
      }
   }

   public boolean solve(MultiContactFrictionBasedSupportRegionSolverInput input, Point2DReadOnly centerOfMassXY)
   {
      numberOfDecisionVariables = basisVectorsPerContactPoint * input.getNumberOfContacts();

      Aeq.reshape(6, numberOfDecisionVariables);
      beq.reshape(6, 1);
      solution = new double[numberOfDecisionVariables];

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).clear();
      }

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         ContactPoint contactPoint = contactPoints.get(i);
         contactPoint.initialize(input);

         FramePoint3D contactPointPosition = input.getContactPointPositions().get(i);

         for (int j = 0; j < basisVectorsPerContactPoint; j++)
         {
            YoFrameVector3D basisVector = contactPoint.getBasisVector(j);
            int column = basisVectorsPerContactPoint * i + j;

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

      beq.set(2, 0, mg);
      beq.set(3, 0, mg * centerOfMassXY.getY());
      beq.set(4, 0, -mg * centerOfMassXY.getX());

      double[] objectiveCoefficients = new double[numberOfDecisionVariables];
      Arrays.fill(objectiveCoefficients, 1.0);
      LinearObjectiveFunction objectiveFunction = new LinearObjectiveFunction(objectiveCoefficients, 0.0);

      List<LinearConstraint> constraints = new ArrayList<>();
      for (int j = 0; j < Aeq.getNumRows(); j++)
      {
         double[] constraintCoefficients = new double[numberOfDecisionVariables];
         for (int k = 0; k < numberOfDecisionVariables; k++)
         {
            constraintCoefficients[k] = Aeq.get(j, k);
         }

         constraints.add(new LinearConstraint(constraintCoefficients, Relationship.EQ, beq.get(j)));
      }

      for (int j = 0; j < numberOfDecisionVariables; j++)
      { // positive rho values
         double[] constraintCoefficients = new double[numberOfDecisionVariables];
         constraintCoefficients[j] = 1.0;
         constraints.add(new LinearConstraint(constraintCoefficients, Relationship.GEQ, 0.0));
      }

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      { // actuation constraint
         double[] constraintCoefficients = new double[numberOfDecisionVariables];
         FrameVector3D normal = input.getSurfaceNormals().get(j);

         for (int rho_i = 0; rho_i < basisVectorsPerContactPoint; rho_i++)
         {
            constraintCoefficients[basisVectorsPerContactPoint * j + rho_i] = contactPoints.get(j).getBasisVector(rho_i).dot(normal);
         }

         double maxNormalForce = input.getActuationConstraints().get(j).getMaxNormalForce();
         Point3D constraintPlanePoint = new Point3D();
         constraintPlanePoint.set(normal);
         constraintPlanePoint.scale(maxNormalForce);

         constraints.add(new LinearConstraint(constraintCoefficients, Relationship.LEQ, constraintPlanePoint.dot(normal)));
      }

      try
      {
         PointValuePair optSolution = solver.optimize(new MaxIter(maximumNumberOfIterations), objectiveFunction, new LinearConstraintSet(constraints), GoalType.MINIMIZE);
         double[] solution = optSolution.getPoint();

         for (int i = 0; i < input.getNumberOfContacts(); i++)
         {
            contactPoints.get(i).setResolvedForce(solution);
         }

         this.solution = Arrays.copyOf(solution, solution.length);
         return true;
      }
      catch (NoFeasibleSolutionException e)
      {
         return false;
      }
   }

   public double[] getSolution()
   {
      return solution;
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
