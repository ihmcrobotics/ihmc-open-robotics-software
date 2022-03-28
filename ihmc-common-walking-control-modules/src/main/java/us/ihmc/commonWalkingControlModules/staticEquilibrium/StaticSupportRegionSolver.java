package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
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
   static final int numberOfDirectionsToOptimize = 32;
   static final double rhoMax = 2.0;
   static final double mass = 1.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

   private final List<StaticEquilibriumContactPoint> contactPoints = new ArrayList<>();
   private StaticEquilibriumSolverInput input;
   private final LinearProgramSolver linearProgramSolver = new LinearProgramSolver();

   private int nominalDecisionVariables;
   private int nonNegativeDecisionVariables;

   /* Nominal equality matrices */
   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);

   /* Expanded equality A matrix, such that all decision variables are non-negative */
   private final DMatrixRMaj APosEq = new DMatrixRMaj(0);

   /* Inequality matrices, which encode the equality matrices */
   private final DMatrixRMaj Ain = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);

   private final DMatrixRMaj costVectorC = new DMatrixRMaj(0);
   private final DMatrixRMaj solution = new DMatrixRMaj(0);

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();
   private final RecyclingArrayList<FramePoint3D> supportRegionVertices = new RecyclingArrayList<>(30, FramePoint3D::new);

   private final YoFramePoint3D averageContactPointPosition = new YoFramePoint3D("averageContactPointPosition", ReferenceFrame.getWorldFrame(), registry);
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

      YoGraphicVector directionToOptimizeGraphic = new YoGraphicVector("directionToOptimizeGraphic", averageContactPointPosition, directionToOptimize, 0.5);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), directionToOptimizeGraphic);

      YoGraphicPosition optimizedCoMGraphic = new YoGraphicPosition("optimizedCoMGraphic", optimizedCoM, 0.03, YoAppearance.DarkRed());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), optimizedCoMGraphic);
   }

   public void initialize(StaticEquilibriumSolverInput input)
   {
      this.input = input;

      int rhoSize = basisVectorsPerContactPoint * input.getNumberOfContacts();
      nominalDecisionVariables = rhoSize + 2;
      nonNegativeDecisionVariables = nominalDecisionVariables + 2;

      Aeq.reshape(6, nominalDecisionVariables);
      beq.reshape(6, 1);
      APosEq.reshape(6, nonNegativeDecisionVariables);
      Ain.reshape(12 + rhoSize, nonNegativeDecisionVariables);
      bin.reshape(12 + rhoSize, 1);
      costVectorC.reshape(nonNegativeDecisionVariables, 1);

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

      Aeq.set(3, nominalDecisionVariables - 1, - mass * input.getGravityMagnitude());
      Aeq.set(4, nominalDecisionVariables - 2, mass * input.getGravityMagnitude());
      beq.set(2, 0, mass * input.getGravityMagnitude());

      MatrixTools.setMatrixBlock(APosEq, 0, 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(APosEq, 0, Aeq.getNumCols(), Aeq, 0, Aeq.getNumCols() - 2, Aeq.getNumRows(), 2, -1.0);

      Arrays.fill(Ain.getData(), 0.0);
      Arrays.fill(bin.getData(), 0.0);

      MatrixTools.setMatrixBlock(Ain, 0, 0, APosEq, 0, 0, 6, APosEq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(Ain, 6, 0, APosEq, 0, 0, 6, APosEq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(bin, 0, 0, beq, 0, 0, 6, beq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, 6, 0, beq, 0, 0, 6, beq.getNumCols(), -1.0);

      for (int i = 0; i < rhoSize; i++)
      {
         Ain.set(12 + i, i, 1.0);
         bin.set(12 + i, 0, rhoMax);
      }

      if (tickAndUpdatable == null)
      {
         return;
      }

      averageContactPointPosition.setToZero();
      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         averageContactPointPosition.add(input.getContactPointPositions().get(i));
      }
      averageContactPointPosition.scale(1.0 / input.getNumberOfContacts());
   }

   public void solve()
   {
      supportRegion.clear();
      supportRegionVertices.clear();
      Arrays.fill(costVectorC.getData(), 0.0);

      for (int i = 0; i < directionsToOptimize.size(); i++)
      {
         Vector2D directionToOptimize = directionsToOptimize.get(i);
         this.directionToOptimize.set(directionToOptimize);

         costVectorC.set(nonNegativeDecisionVariables - 4, 0, directionToOptimize.getX());
         costVectorC.set(nonNegativeDecisionVariables - 3, 0, directionToOptimize.getY());
         costVectorC.set(nonNegativeDecisionVariables - 2, 0, -directionToOptimize.getX());
         costVectorC.set(nonNegativeDecisionVariables - 1, 0, -directionToOptimize.getY());

         linearProgramSolver.solve(costVectorC, Ain, bin, solution);

         double comExtremumX = solution.get(nonNegativeDecisionVariables - 4) - solution.get(nonNegativeDecisionVariables - 2);
         double comExtremumY = solution.get(nonNegativeDecisionVariables - 3) - solution.get(nonNegativeDecisionVariables - 1);
         supportRegion.addVertex(comExtremumX, comExtremumY);
         supportRegionVertices.add().setIncludingFrame(ReferenceFrame.getWorldFrame(), comExtremumX, comExtremumY, 0.0);

         updateGraphics();
      }

      supportRegion.update();
   }

   private void updateGraphics()
   {
      if (tickAndUpdatable == null)
         return;

      for (int j = 0; j < input.getNumberOfContacts(); j++)
      {
         contactPoints.get(j).setResolvedForce(solution.getData());
      }

      double comExtremumX = solution.get(nonNegativeDecisionVariables - 4) - solution.get(nonNegativeDecisionVariables - 2);
      double comExtremumY = solution.get(nonNegativeDecisionVariables - 3) - solution.get(nonNegativeDecisionVariables - 1);

      this.optimizedCoM.setX(comExtremumX);
      this.optimizedCoM.setY(comExtremumY);
      this.optimizedCoM.setZ(averageContactPointPosition.getZ());

      tickAndUpdatable.tickAndUpdate();
   }

   //////////////////////////////////////////////////////////////////////////////////////////
   ////////////////////////////////////////  GETTERS ////////////////////////////////////////
   //////////////////////////////////////////////////////////////////////////////////////////

   public ConvexPolygon2DReadOnly getSupportRegion()
   {
      return supportRegion;
   }

   public RecyclingArrayList<? extends FramePoint3DReadOnly> getSupportRegionVertices()
   {
      return supportRegionVertices;
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

   public YoFramePoint3D getAverageContactPointPosition()
   {
      return averageContactPointPosition;
   }
}
