package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.linear.*;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.StaticEquilibriumContactPoint.basisVectorsPerContactPoint;

/**
 * This is an implementation of "Testing Static Equilibrium for Legged Robots", Bretl et al, 2008
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 *
 * It solves for the (convex) region of feasible CoM XY positions, modelling the robot as a point mass
 * and imposing friction constraints.
 */
public class StaticSupportRegionSolver
{
   private static final int numberOfDirectionsToOptimize = 32;
   private static final int maximumNumberOfIterations = 10000;
   private static final double convergenceThreshold = 1e-5;
   private static final double rhoMax = 10.0;
   /* Only dimensionally related to rhoMax. */
   static final double mass = 1.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

   private final List<StaticEquilibriumContactPoint> contactPoints = new ArrayList<>();

   private StaticEquilibriumSolverInput input;
   private final SimplexSolver solver = new SimplexSolver(convergenceThreshold);
   private int numberOfDecisionVariables;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 0);

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();
   private final List<Point2D> points = new ArrayList<>();

   private final YoFramePoint3D directionToOptimizeBase = new YoFramePoint3D("directionToOptimizeBase", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D directionToOptimize = new YoFrameVector3D("directionToOptimize", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);

   private static final List<Vector2D> directionsToOptimize = new ArrayList<>();

   static
   {
      double dTheta = 2.0 * Math.PI / numberOfDirectionsToOptimize;
      for (int i = 0; i < numberOfDirectionsToOptimize; i++)
      {
         directionsToOptimize.add(new Vector2D(Math.cos(i * dTheta), Math.sin(i * dTheta)));
      }
   }

   public StaticSupportRegionSolver()
   {
      for (int i = 0; i < StaticEquilibriumSolverInput.maxContactPoints; i++)
      {
         contactPoints.add(new StaticEquilibriumContactPoint(i, registry, graphicsListRegistry));
      }

      YoGraphicVector directionToOptimizeGraphic = new YoGraphicVector("directionToOptimizeGraphic", directionToOptimizeBase, directionToOptimize, 0.5);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), directionToOptimizeGraphic);
      directionToOptimizeBase.set(0.0, 0.0, 0.4);

      YoGraphicPosition optimizedCoMGraphic = new YoGraphicPosition("optimizedCoMGraphic", optimizedCoM, 0.03, YoAppearance.DarkRed());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), optimizedCoMGraphic);
   }

   public void initialize(StaticEquilibriumSolverInput input)
   {
      this.input = input;
      numberOfDecisionVariables = basisVectorsPerContactPoint * input.getNumberOfContacts() + 2;
      points.clear();

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

      Aeq.set(3, numberOfDecisionVariables - 1, - mass * input.getGravityMagnitude());
      Aeq.set(4, numberOfDecisionVariables - 2, mass * input.getGravityMagnitude());
      beq.set(2, 0, mass * input.getGravityMagnitude());
   }

   public void solve()
   {
      supportRegion.clear();

      for (int i = 0; i < directionsToOptimize.size(); i++)
      {
         Vector2D directionToOptimize = directionsToOptimize.get(i);
         this.directionToOptimize.set(directionToOptimize);

         double[] objectiveCoefficients = new double[numberOfDecisionVariables];
         objectiveCoefficients[numberOfDecisionVariables - 2] = directionToOptimize.getX();
         objectiveCoefficients[numberOfDecisionVariables - 1] = directionToOptimize.getY();
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

         for (int j = 0; j < numberOfDecisionVariables - 2; j++)
         {
            double[] constraintCoefficients = new double[numberOfDecisionVariables];
            constraintCoefficients[j] = 1.0;
            constraints.add(new LinearConstraint(constraintCoefficients, Relationship.GEQ, 0.0));
            constraints.add(new LinearConstraint(constraintCoefficients, Relationship.LEQ, rhoMax));
         }

         PointValuePair optSolution = solver.optimize(new MaxIter(maximumNumberOfIterations), objectiveFunction, new LinearConstraintSet(constraints), GoalType.MAXIMIZE);
         double[] solution = optSolution.getPoint();
         double comExtremumX = solution[numberOfDecisionVariables - 2];
         double comExtremumY = solution[numberOfDecisionVariables - 1];
         supportRegion.addVertex(comExtremumX, comExtremumY);
         points.add(new Point2D(comExtremumX, comExtremumY));
         updateGraphics(solution);
      }

      supportRegion.update();
   }

   private void updateGraphics(double[] solution)
   {
      if (tickAndUpdatable == null)
         return;

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      {
         contactPoints.get(j).setResolvedForce(solution);
      }

      this.optimizedCoM.setX(solution[numberOfDecisionVariables - 2]);
      this.optimizedCoM.setY(solution[numberOfDecisionVariables - 1]);
      this.optimizedCoM.setZ(0.1);

      tickAndUpdatable.tickAndUpdate();
   }

   //////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////////////////////  GETTERS ////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////

   public ConvexPolygon2D getSupportRegion()
   {
      supportRegion.update();
      return new ConvexPolygon2D(supportRegion);
   }

   public List<Point2D> getPoints()
   {
      return points;
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

   public DMatrixRMaj getAeq()
   {
      return Aeq;
   }

   public DMatrixRMaj getBeq()
   {
      return beq;
   }
}
