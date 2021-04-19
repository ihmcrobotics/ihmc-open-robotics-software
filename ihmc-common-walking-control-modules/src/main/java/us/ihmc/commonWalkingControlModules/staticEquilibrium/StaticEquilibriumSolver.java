package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolverWithInactiveVariables;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Arrays;
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
   private static final boolean updateGraphicsEachIteration = false;
   private static final int numberOfDirectionsToOptimize = 16;
   private static final int maximumNumberOfIterations = 8000;
   private static final double convergenceThreshold = 1e-7;
   private static final double rhoMax = 20.0;
   private static final double alphaQuadraticCost = 10.0e-2;
   static final double mass = 1.0;

   private enum Mode {CUSTOM, QP}
   private static final Mode mode = Mode.CUSTOM;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

   private final List<StaticEquilibriumContactPoint> contactPoints = new ArrayList<>();

   private StaticEquilibriumSolverInput input;
   private int numberOfDecisionVariables;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 0);

//   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();
   private final SimpleEfficientActiveSetQPSolverWithInactiveVariables qpSolver = new SimpleEfficientActiveSetQPSolverWithInactiveVariables();

   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj lowerBound = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj upperBound = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Ain = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj AeqActive = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj activeDiagonal = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj xTrial = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj xSolution = new DMatrixRMaj(0, 0);

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();

   private final YoFramePoint3D directionToOptimizeBase = new YoFramePoint3D("directionToOptimizeBase", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D directionToOptimize = new YoFrameVector3D("directionToOptimize", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D candidateCoM = new YoFramePoint3D("candidateCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);

   private final YoInteger iterations = new YoInteger("iterations", registry);

   private static final List<Vector2D> directionsToOptimize = new ArrayList<>();

   static
   {
      double dTheta = 2.0 * Math.PI / numberOfDirectionsToOptimize;
      for (int i = 0; i < numberOfDirectionsToOptimize; i++)
      {
         directionsToOptimize.add(new Vector2D(Math.cos(i * dTheta), Math.sin(i * dTheta)));
      }

//      directionsToOptimize.add(new Vector2D(1.0, 0.0));

//      double theta = 0.3;
//      directionsToOptimize.add(new Vector2D(Math.cos(theta), Math.sin(theta)));
   }

   public StaticEquilibriumSolver()
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

      YoGraphicPosition candidateCoMGraphic = new YoGraphicPosition("candidateCoM", candidateCoM, 0.03, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), candidateCoMGraphic);
   }

   public void initialize(StaticEquilibriumSolverInput input)
   {
      this.input = input;
      numberOfDecisionVariables = 4 * input.getNumberOfContacts() + 2;

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

      Aeq.set(3, numberOfDecisionVariables - 1, - mass * input.getGravityMagnitude());
      Aeq.set(4, numberOfDecisionVariables - 2, mass * input.getGravityMagnitude());
      beq.set(2, 0, mass * input.getGravityMagnitude());

      xTrial.reshape(numberOfDecisionVariables, 1);
      xSolution.reshape(numberOfDecisionVariables, 1);

      if (mode == Mode.CUSTOM)
      {
         AeqActive.reshape(6, numberOfDecisionVariables);
         activeDiagonal.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
         CommonOps_DDRM.setIdentity(activeDiagonal);
      }
      else if (mode == Mode.QP)
      {
         quadraticCost.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
         linearCost.reshape(numberOfDecisionVariables, 1);
         lowerBound.reshape(numberOfDecisionVariables, 1);
         upperBound.reshape(numberOfDecisionVariables, 1);
         Ain.reshape(numberOfDecisionVariables - 2, numberOfDecisionVariables);
         bin.reshape(numberOfDecisionVariables - 2, 1);
         CommonOps_DDRM.fill(lowerBound, 0.0);
         CommonOps_DDRM.fill(upperBound, 100.0);
         lowerBound.set(numberOfDecisionVariables - 2, -100.0);
         lowerBound.set(numberOfDecisionVariables - 1, -100.0);
         CommonOps_DDRM.fill(bin, -1e-5);

         for (int i = 0; i < numberOfDecisionVariables - 2; i++)
         {
            Ain.set(i, i, -1.0);
         }

         CommonOps_DDRM.setIdentity(quadraticCost);
         CommonOps_DDRM.scale(alphaQuadraticCost, quadraticCost);
      }
   }

   public void solve()
   {
      supportRegion.clear();
      for (int i = 0; i < directionsToOptimize.size(); i++)
      {
         Vector2D directionToOptimize = directionsToOptimize.get(i);
         this.directionToOptimize.set(directionToOptimize);

         switch (mode)
         {
            case QP:
               linearCost.set(numberOfDecisionVariables - 2, -100.0 * directionToOptimize.getX());
               linearCost.set(numberOfDecisionVariables - 1, -100.0 * directionToOptimize.getY());

               qpSolver.clear();
               qpSolver.resetActiveSet();
               qpSolver.setUseWarmStart(false);

               qpSolver.setConvergenceThreshold(1e-8);
               qpSolver.setMaxNumberOfIterations(400);

               qpSolver.setQuadraticCostFunction(quadraticCost, linearCost);
               qpSolver.setLinearEqualityConstraints(Aeq, beq);
               qpSolver.setVariableBounds(lowerBound, upperBound);
               qpSolver.setLinearInequalityConstraints(Ain, bin);
               qpSolver.solve(xSolution);
               break;
            case CUSTOM:
            default:
               computeExtremeFeasibleCoM(directionToOptimize);
         }

         setFinalGraphics();
         double comExtremumX = xSolution.get(numberOfDecisionVariables - 2, 0);
         double comExtremumY = xSolution.get(numberOfDecisionVariables - 1, 0);
         supportRegion.addVertex(comExtremumX, comExtremumY);
      }
      supportRegion.update();
   }

   //////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////  CUSTOM SOLVER /////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////

   private final YoDouble deltaCoM = new YoDouble("deltaCoM", registry);
   private final YoInteger activeBetaSize = new YoInteger("activeBetaSize", registry);

   // projection of z onto Aeq x = beq   >>>>   p0*z + p1
   private final DMatrixRMaj p0 = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj p1 = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj xProjected = new DMatrixRMaj(0, 0);

   private void computeExtremeFeasibleCoM(Vector2D directionToOptimize)
   {
      iterations.set(0);
      CommonOps_DDRM.setIdentity(activeDiagonal);
      computeProjectionMatrices();
      computeInitialInteriorPoint();
      xSolution.set(xTrial);

      boolean[] activeBetas = new boolean[numberOfDecisionVariables - 2];
      Arrays.fill(activeBetas, true);

      while (iterations.getValue() < maximumNumberOfIterations)
      {
         iterations.increment();

         if (iterations.getIntegerValue() > 1 && terminate())
            break;

         // check friction constraints
         boolean foundInvalidFrictionConstraint = false;
         for (int i = 0; i < numberOfDecisionVariables - 2; i++)
         {
            if (xTrial.get(i) < 0.0)
            {
               foundInvalidFrictionConstraint = true;
               xTrial.set(i, 1.0e-5);
               activeDiagonal.set(i, i, 0.0);
               activeBetas[i] = false;
            }
            else if (xTrial.get(i) > rhoMax)
            {
               foundInvalidFrictionConstraint = true;
               // scale down slightly
               xTrial.set(i, rhoMax * 0.95);
            }
         }

         if (foundInvalidFrictionConstraint)
         {
            computeProjectionMatrices();
         }

         activeBetaSize.set(0);
         for (int i = 0; i < activeBetas.length; i++)
         {
            if (activeBetas[i])
               activeBetaSize.increment();
         }

         if (!foundInvalidFrictionConstraint)
         {
            if (terminate())
               break;
            else
               xSolution.set(xTrial);

            // TODO figure out on-the-fly scaling
            double shiftAmount = 0.3;
            xTrial.add(numberOfDecisionVariables - 2, 0, shiftAmount * directionToOptimize.getX());
            xTrial.add(numberOfDecisionVariables - 1, 0, shiftAmount * directionToOptimize.getY());
         }

         // enforce static equilibrium
         projectToEqualityConstraints(xTrial);

         if (updateGraphicsEachIteration)
            setIntermediateGraphics();
      }
   }

   private void setIntermediateGraphics()
   {
      if (tickAndUpdatable == null)
         return;

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      {
         contactPoints.get(j).setResolvedForce(xTrial);
      }

      this.optimizedCoM.setX(xSolution.get(numberOfDecisionVariables - 2));
      this.optimizedCoM.setY(xSolution.get(numberOfDecisionVariables - 1));
      this.optimizedCoM.setZ(0.1);

      this.candidateCoM.setX(xTrial.get(numberOfDecisionVariables - 2));
      this.candidateCoM.setY(xTrial.get(numberOfDecisionVariables - 1));
      this.candidateCoM.setZ(0.1);

      tickAndUpdatable.tickAndUpdate();
   }

   private void setFinalGraphics()
   {
      if (tickAndUpdatable == null)
         return;

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      {
         contactPoints.get(j).setResolvedForce(xSolution);
      }

      this.candidateCoM.setToNaN();
      this.optimizedCoM.setX(xSolution.get(numberOfDecisionVariables - 2));
      this.optimizedCoM.setY(xSolution.get(numberOfDecisionVariables - 1));
      this.optimizedCoM.setZ(0.1);

      tickAndUpdatable.tickAndUpdate();
   }
   private void computeProjectionMatrices()
   {
      CommonOps_DDRM.mult(Aeq, activeDiagonal, AeqActive);

      DMatrixRMaj AATinv = new DMatrixRMaj(0, 0);
      CommonOps_DDRM.multOuter(AeqActive, AATinv);
      CommonOps_DDRM.invert(AATinv);

      DMatrixRMaj ATAATinv = new DMatrixRMaj(0, 0);
      CommonOps_DDRM.multTransA(AeqActive, AATinv, ATAATinv);
      CommonOps_DDRM.mult(ATAATinv, beq, p1);

      CommonOps_DDRM.mult(ATAATinv, AeqActive, p0);
      CommonOps_DDRM.scale(-1.0, p0);
      CommonOps_DDRM.addEquals(p0, CommonOps_DDRM.identity(numberOfDecisionVariables));
   }
   private boolean terminate()
   {
      if (iterations.getIntegerValue() <= 1)
         return false;

      if (activeBetaSize.getValue() <= 4)
      {
         if (updateGraphicsEachIteration)
            System.out.println("Terminating. Num active betas: " + activeBetaSize);
         return true;
      }

      double pX = xTrial.get(numberOfDecisionVariables - 2);
      double pY = xTrial.get(numberOfDecisionVariables - 1);
      double bestX = xSolution.get(numberOfDecisionVariables - 2);
      double bestY = xSolution.get(numberOfDecisionVariables - 1);
      double distance = EuclidCoreTools.norm(pX - bestX, pY - bestY);
      deltaCoM.set(Math.sqrt(distance));
      boolean hasConverged = distance < convergenceThreshold;

      if (hasConverged)
      {
         if (updateGraphicsEachIteration)
            System.out.println("Terminating. Delta com: " + distance);
      }

      return hasConverged;
   }

   private void computeInitialInteriorPoint()
   {
      Point2D averageContactPointXY = new Point2D();
      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         averageContactPointXY.add(input.getContactPointPositions().get(i).getX(), input.getContactPointPositions().get(i).getY());
      }
      averageContactPointXY.scale(1.0 / input.getNumberOfContacts());

      // guess for flat ground
      double flatGroundBetaZ = 1.0 / Math.sqrt(1.0 + MathTools.square(input.getCoefficientOfFriction()));
      double rhoGuess = mass * input.getGravityMagnitude() / (4.0 * input.getNumberOfContacts() * flatGroundBetaZ);
      CommonOps_DDRM.fill(xTrial, rhoGuess);

      // set com to center of contacts
      xTrial.set(numberOfDecisionVariables - 2, averageContactPointXY.getX());
      xTrial.set(numberOfDecisionVariables - 1, averageContactPointXY.getY());

      projectToEqualityConstraints(xTrial);
   }

   private void projectToEqualityConstraints(DMatrixRMaj x)
   {
      CommonOps_DDRM.mult(p0, x, xProjected);
      CommonOps_DDRM.addEquals(xProjected, p1);
      x.set(xProjected);
   }

   //////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////////////////////  GETTERS ////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////

   public ConvexPolygon2D getSupportRegion()
   {
      supportRegion.update();
      return new ConvexPolygon2D(supportRegion);
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
