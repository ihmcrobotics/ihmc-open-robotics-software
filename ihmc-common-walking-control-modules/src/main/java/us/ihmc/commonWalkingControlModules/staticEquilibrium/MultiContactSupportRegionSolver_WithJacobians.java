package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is an implementation of "Testing Static Equilibrium for Legged Robots", Bretl et al, 2008
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 *
 * It solves for the convex region of feasible CoM XY positions, modelling the robot as a point mass and imposing friction constraints.
 */
public class MultiContactSupportRegionSolver_WithJacobians
{
   private static final double optimizedCoMGraphicScale = 0.03;

   private static final int defaultNumberOfDirectionsToOptimize = 20;
   private static final int centerOfMassDimensions = 2;
   private static final int staticEquilibriumConstraints = 6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

   private final List<ContactPoint> contactPoints = new ArrayList<>();
   private MultiContactForceDistributionInput input;
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

   private final Point3D actuationConstraintCentroid = new Point3D();

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();
   private final RecyclingArrayList<FramePoint3D> supportRegionVertices = new RecyclingArrayList<>(30, FramePoint3D::new);

   private final YoFramePoint3D averageContactPointPosition = new YoFramePoint3D("averageContactPointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D directionToOptimize = new YoFrameVector3D("directionToOptimize", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);
   private final List<Vector2D> directionsToOptimize = new ArrayList<>();
   private boolean foundSolution = false;

   private final ExecutionTimer executionTimer;
   private final List<Point2D> vertexList = new ArrayList<>();

   public MultiContactSupportRegionSolver_WithJacobians()
   {
      this(defaultNumberOfDirectionsToOptimize);
   }

   public MultiContactSupportRegionSolver_WithJacobians(int numberOfDirectionsToOptimize)
   {
      this(numberOfDirectionsToOptimize, null);
   }

   public MultiContactSupportRegionSolver_WithJacobians(int numberOfDirectionsToOptimize, YoRegistry registry)
   {
      executionTimer = registry == null ? null : new ExecutionTimer("comSolver", registry);

      for (int i = 0; i < MultiContactSupportRegionSolverInput.maxContactPoints; i++)
      {
         contactPoints.add(new ContactPoint(i, registry, graphicsListRegistry));
      }

      setNumberOfDirectionsToOptimize(numberOfDirectionsToOptimize);

      YoGraphicVector directionToOptimizeGraphic = new YoGraphicVector("directionToOptimizeGraphic", averageContactPointPosition, directionToOptimize, 0.5);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), directionToOptimizeGraphic);

      YoGraphicPosition optimizedCoMGraphic = new YoGraphicPosition("optimizedCoMGraphic", optimizedCoM, optimizedCoMGraphicScale, YoAppearance.DarkRed());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), optimizedCoMGraphic);
   }

   public void initialize(MultiContactForceDistributionInput input)
   {
      clear();

      double mg = 9.81 * input.getRobotMass();
      this.input = input;

      int rhoSize = MultiContactForceDistributionInput.numberOfBasisVectors * input.getNumberOfContactPoints();
      nominalDecisionVariables = rhoSize + centerOfMassDimensions;
      nonNegativeDecisionVariables = nominalDecisionVariables + centerOfMassDimensions;
      int actuationConstraints = input.getNumberOfJoints();

      Aeq.reshape(staticEquilibriumConstraints, nominalDecisionVariables);
      beq.reshape(staticEquilibriumConstraints, 1);
      APosEq.reshape(staticEquilibriumConstraints, nonNegativeDecisionVariables);
      Ain.reshape(2 * staticEquilibriumConstraints + 2 * actuationConstraints, nonNegativeDecisionVariables);
      bin.reshape(2 * staticEquilibriumConstraints + 2 * actuationConstraints, 1);
      costVectorC.reshape(nonNegativeDecisionVariables, 1);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).clear();
      }

      for (int contactPointIndex = 0; contactPointIndex < input.getNumberOfContactPoints(); contactPointIndex++)
      {
         FramePoint3D contactPointPosition = new FramePoint3D(input.getContactFrame(contactPointIndex));
         contactPointPosition.changeFrame(ReferenceFrame.getWorldFrame());

         for (int basisVectorIndex = 0; basisVectorIndex < MultiContactForceDistributionInput.numberOfBasisVectors; basisVectorIndex++)
         {
            FrameVector3D basisVector = input.getBasisVector(contactPointIndex, basisVectorIndex);
            basisVector.changeFrame(ReferenceFrame.getWorldFrame());
            int column = MultiContactForceDistributionInput.numberOfBasisVectors * contactPointIndex + basisVectorIndex;

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

      Aeq.set(3, nominalDecisionVariables - 1, -mg);
      Aeq.set(4, nominalDecisionVariables - 2, mg);
      beq.set(2, 0, mg);

      MatrixTools.setMatrixBlock(APosEq, 0, 0, Aeq, 0, 0, Aeq.getNumRows(), Aeq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(APosEq, 0, Aeq.getNumCols(), Aeq, 0, Aeq.getNumCols() - 2, Aeq.getNumRows(), 2, -1.0);

      Arrays.fill(Ain.getData(), 0.0);
      Arrays.fill(bin.getData(), 0.0);

      MatrixTools.setMatrixBlock(Ain, 0, 0, APosEq, 0, 0, APosEq.getNumRows(), APosEq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(Ain, APosEq.getNumRows(), 0, APosEq, 0, 0, APosEq.getNumRows(), APosEq.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(bin, 0, 0, beq, 0, 0, APosEq.getNumRows(), beq.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, APosEq.getNumRows(), 0, beq, 0, 0, APosEq.getNumRows(), beq.getNumCols(), -1.0);

      // Add actuation constraints
      DMatrixRMaj constraintUpperBound = input.getConstraintUpperBound();
      DMatrixRMaj constraintLowerBound = input.getConstraintLowerBound();
      DMatrixRMaj graspMatrixJacobianTranspose = input.getGraspMatrixJacobianTranspose();

      MatrixTools.setMatrixBlock(Ain, 2 * staticEquilibriumConstraints, 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(bin, 2 * staticEquilibriumConstraints, 0, constraintUpperBound, 0, 0, constraintUpperBound.getNumRows(), constraintUpperBound.getNumCols(), 1.0);
      MatrixTools.setMatrixBlock(Ain, 2 * staticEquilibriumConstraints + graspMatrixJacobianTranspose.getNumRows(), 0, graspMatrixJacobianTranspose, 0, 0, graspMatrixJacobianTranspose.getNumRows(), graspMatrixJacobianTranspose.getNumCols(), -1.0);
      MatrixTools.setMatrixBlock(bin, 2 * staticEquilibriumConstraints + graspMatrixJacobianTranspose.getNumRows(), 0, constraintLowerBound, 0, 0, constraintUpperBound.getNumRows(), constraintUpperBound.getNumCols(), -1.0);
   }

   public void setNumberOfDirectionsToOptimize(int numberOfDirectionsToOptimize)
   {
      if (numberOfDirectionsToOptimize == directionsToOptimize.size())
      {
         return;
      }

      directionsToOptimize.clear();
      double deltaAngle = 2.0 * Math.PI / numberOfDirectionsToOptimize;
      for (int i = 0; i < numberOfDirectionsToOptimize; i++)
      {
         directionsToOptimize.add(new Vector2D(Math.cos(i * deltaAngle), Math.sin(i * deltaAngle)));
      }
   }

   public boolean solve()
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

         foundSolution = linearProgramSolver.solve(costVectorC, Ain, bin, solution);
         if (!foundSolution)
         {
            supportRegion.clear();
            return false;
         }

         double comExtremumX = solution.get(nonNegativeDecisionVariables - 4) - solution.get(nonNegativeDecisionVariables - 2);
         double comExtremumY = solution.get(nonNegativeDecisionVariables - 3) - solution.get(nonNegativeDecisionVariables - 1);
         supportRegion.addVertex(comExtremumX, comExtremumY);
         supportRegionVertices.add().setIncludingFrame(ReferenceFrame.getWorldFrame(), comExtremumX, comExtremumY, 0.0);

         TIntArrayList activeSet = new TIntArrayList();
         TIntArrayList activeSetIndicesSolverObj = linearProgramSolver.getSimplexStatistics().getActiveSetIndices();
         for (int j = 0; j < activeSetIndicesSolverObj.size(); j++)
         {
            if (activeSetIndicesSolverObj.get(j) >= 2 * staticEquilibriumConstraints)
            {
               activeSet.add(activeSetIndicesSolverObj.get(j) - 2 * staticEquilibriumConstraints);
            }
         }

         vertexList.add(new Point2D(comExtremumX, comExtremumY));
      }

      supportRegion.update();

      for (int i = 0; i < supportRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = supportRegion.getVertex(i);
         boolean foundMatch = false;

         for (int j = 0; j < vertexList.size(); j++)
         {
            if (vertexList.get(j).epsilonEquals(vertex, 1e-4))
            {
               foundMatch = true;
               break;
            }
         }

         if (!foundMatch)
         {
            LogTools.error("Couldn't find match for vertex " + i);
         }
      }

      return true;
   }

   public boolean foundSolution()
   {
      return foundSolution;
   }

   private void clear()
   {
      Aeq.zero();
      beq.zero();
      APosEq.zero();
      Ain.zero();
      bin.zero();
      costVectorC.zero();
      solution.zero();

      vertexList.clear();
   }

   public void startTimer()
   {
      if (executionTimer != null)
         executionTimer.startMeasurement();
   }

   public void stopTimer()
   {
      if (executionTimer != null)
         executionTimer.stopMeasurement();
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
