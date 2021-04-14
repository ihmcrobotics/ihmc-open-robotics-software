package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
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
import us.ihmc.yoVariables.variable.YoBoolean;
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
   private static final int maximumNumberOfIterations = 30000;
   private static final double convergenceThreshold = 1e-6;
   private static final double rhoMaxForUnitMass = 10.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

   private final List<StaticEquilibriumContactPoint> contactPoints = new ArrayList<>();

   private StaticEquilibriumSolverInput input;
   private int numberOfDecisionVariables;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj AeqActive = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj activeDiagonal = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj x = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj xBestFeasible = new DMatrixRMaj(0, 0);

   private final List<Point2D> supportRegion = new ArrayList<>();

   private final YoFramePoint3D directionToOptimizeBase = new YoFramePoint3D("directionToOptimizeBase", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D directionToOptimize = new YoFrameVector3D("directionToOptimize", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D candidateCoM = new YoFramePoint3D("candidateCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);

   private final YoInteger iterations = new YoInteger("iterations", registry);
   private final YoInteger activeBetaSize = new YoInteger("activeBetaSize", registry);
   private final YoDouble rhoMax = new YoDouble("rhoMax", registry);
   private final YoDouble deltaCoM = new YoDouble("deltaCoM", registry);

   // projection of z onto Aeq x = beq   >>>>   p0*z + p1
   private final DMatrixRMaj p0 = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj p1 = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj xProjected = new DMatrixRMaj(0, 0);

   private static final List<Vector2D> directionsToOptimize = new ArrayList<>();

   static
   {
      double dTheta = 2.0 * Math.PI / numberOfDirectionsToOptimize;
      for (int i = 0; i < numberOfDirectionsToOptimize; i++)
      {
         directionsToOptimize.add(new Vector2D(Math.cos(i * dTheta), Math.sin(i * dTheta)));
      }

//      double theta = Math.toRadians(200.0);
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

   public void solve(StaticEquilibriumSolverInput input)
   {
      if (!input.checkInput())
      {
         System.out.println("Invalid input");
         return;
      }

      this.input = input;
      int numberOfContactPoints = input.getNumberOfContacts();
      numberOfDecisionVariables = 4 * numberOfContactPoints + 2;
      rhoMax.set(input.getRobotMass() * rhoMaxForUnitMass);

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

      x.reshape(numberOfDecisionVariables, 1);
      xBestFeasible.reshape(numberOfDecisionVariables, 1);

      AeqActive.reshape(6, numberOfDecisionVariables);
      activeDiagonal.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      CommonOps_DDRM.setIdentity(activeDiagonal);

      for (int i = 0; i < directionsToOptimize.size(); i++)
      {
         Vector2D directionToOptimize = directionsToOptimize.get(i);
         this.directionToOptimize.set(directionToOptimize);

         computeExtremeFeasibleCoM(directionToOptimize);

         setFinalGraphics();
         double comExtremumX = xBestFeasible.get(numberOfDecisionVariables - 2, 0);
         double comExtremumY = xBestFeasible.get(numberOfDecisionVariables - 1, 0);
         supportRegion.add(new Point2D(comExtremumX, comExtremumY));
      }
   }

   private void setIntermediateGraphics()
   {
      if (tickAndUpdatable == null)
         return;

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      {
         contactPoints.get(j).setResolvedForce(x);
      }

      this.optimizedCoM.setX(xBestFeasible.get(numberOfDecisionVariables - 2));
      this.optimizedCoM.setY(xBestFeasible.get(numberOfDecisionVariables - 1));
      this.optimizedCoM.setZ(0.1);

      this.candidateCoM.setX(x.get(numberOfDecisionVariables - 2));
      this.candidateCoM.setY(x.get(numberOfDecisionVariables - 1));
      this.candidateCoM.setZ(0.1);

      tickAndUpdatable.tickAndUpdate();
   }

   private void setFinalGraphics()
   {
      if (tickAndUpdatable == null)
         return;

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      {
         contactPoints.get(j).setResolvedForce(xBestFeasible);
      }

      this.candidateCoM.setToNaN();
      this.optimizedCoM.setX(xBestFeasible.get(numberOfDecisionVariables - 2));
      this.optimizedCoM.setY(xBestFeasible.get(numberOfDecisionVariables - 1));
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

   private void computeExtremeFeasibleCoM(Vector2D directionToOptimize)
   {
      iterations.set(0);
      CommonOps_DDRM.setIdentity(activeDiagonal);
      computeProjectionMatrices();
      computeInitialInteriorPoint();
      xBestFeasible.set(x);

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
            if (x.get(i) < 0.0)
            {
               foundInvalidFrictionConstraint = true;
               x.set(i, 1.0e-5);
               activeDiagonal.set(i, i, 0.0);
               activeBetas[i] = false;
            }
            else if (x.get(i) > rhoMax.getValue())
            {
               foundInvalidFrictionConstraint = true;
               // scale down slightly
               x.set(i, rhoMax.getValue() * 0.95);
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
               xBestFeasible.set(x);

            double shiftAmount = 20.0 * input.getRobotMass();
            x.add(numberOfDecisionVariables - 2, 0, shiftAmount * directionToOptimize.getX());
            x.add(numberOfDecisionVariables - 1, 0, shiftAmount * directionToOptimize.getY());
         }

         // enforce static equilibrium
         projectToEqualityConstraints(x);

         if (updateGraphicsEachIteration)
            setIntermediateGraphics();
      }
   }

   private boolean terminate()
   {
      if (iterations.getIntegerValue() <= 1)
         return false;

      if (activeBetaSize.getValue() <= 4)
         return true;

      double pX = x.get(numberOfDecisionVariables - 2);
      double pY = x.get(numberOfDecisionVariables - 1);
      double bestX = xBestFeasible.get(numberOfDecisionVariables - 2);
      double bestY = xBestFeasible.get(numberOfDecisionVariables - 1);
      double distance = EuclidCoreTools.norm(pX - bestX, pY - bestY);
      deltaCoM.set(Math.sqrt(distance));

      return distance < convergenceThreshold;
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
      double rhoGuess = input.getRobotMass() * input.getGravityMagnitude() / (4.0 * input.getNumberOfContacts() * flatGroundBetaZ);
      CommonOps_DDRM.fill(x, rhoGuess);

      // set com to center of contacts
      x.set(numberOfDecisionVariables - 2, averageContactPointXY.getX());
      x.set(numberOfDecisionVariables - 1, averageContactPointXY.getY());

      projectToEqualityConstraints(x);
   }

   private void projectToEqualityConstraints(DMatrixRMaj x)
   {
      CommonOps_DDRM.mult(p0, x, xProjected);
      CommonOps_DDRM.addEquals(xProjected, p1);
      x.set(xProjected);
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
