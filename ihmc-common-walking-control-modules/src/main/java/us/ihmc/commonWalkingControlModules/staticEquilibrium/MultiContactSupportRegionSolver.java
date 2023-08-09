package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.ContactPoint.basisVectorsPerContactPoint;

/**
 * This is an implementation of "Testing Static Equilibrium for Legged Robots", Bretl et al, 2008
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 * <p>
 * It solves for the convex region of feasible CoM XY positions, modelling the robot as a point mass and imposing friction constraints.
 */
public class MultiContactSupportRegionSolver
{
   private static final double optimizedCoMGraphicScale = 0.03;

   private static final int defaultNumberOfDirectionsToOptimize = 4;
   private static final int centerOfMassDimensions = 2;
   private static final int staticEquilibriumConstraints = 6;
   static final double mg = 1.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private TickAndUpdatable tickAndUpdatable = null;

   private final List<ContactPoint> contactPoints = new ArrayList<>();
   private MultiContactSupportRegionSolverInput input;
   private final LinearProgramSolver linearProgramSolver = new LinearProgramSolver();
   private final YoBoolean foundSolution = new YoBoolean("foundSolution", registry);

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
   private final RecyclingArrayList<FramePoint3D> expandedSupportRegionVertices = new RecyclingArrayList<>(30, FramePoint3D::new);

   private final YoFramePoint3D averageContactPointPosition = new YoFramePoint3D("averageContactPointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D directionToOptimize = new YoFrameVector3D("directionToOptimize", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D optimizedCoM = new YoFramePoint3D("optimizedCoM", ReferenceFrame.getWorldFrame(), registry);
   private final List<Vector2D> directionsToOptimize = new ArrayList<>();

   private final List<Point2D> vertexList = new ArrayList<>();
   private final List<TIntArrayList> activeSetIndices = new ArrayList<>();
   private final List<TIntArrayList> orderedActiveSetIndices = new ArrayList<>();

   public MultiContactSupportRegionSolver()
   {
      this(defaultNumberOfDirectionsToOptimize);
   }

   public MultiContactSupportRegionSolver(int numberOfDirectionsToOptimize)
   {
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

   public void initialize(MultiContactSupportRegionSolverInput input)
   {
      clear();

      this.input = input;

      int rhoSize = basisVectorsPerContactPoint * input.getNumberOfContacts();
      nominalDecisionVariables = rhoSize + centerOfMassDimensions;
      nonNegativeDecisionVariables = nominalDecisionVariables + centerOfMassDimensions;
      int actuationConstraints = 0;
      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         actuationConstraints += input.getActuationConstraints().get(i).getNumberOfConstraints();
      }

      Aeq.reshape(staticEquilibriumConstraints, nominalDecisionVariables);
      beq.reshape(staticEquilibriumConstraints, 1);
      APosEq.reshape(staticEquilibriumConstraints, nonNegativeDecisionVariables);
      Ain.reshape(2 * staticEquilibriumConstraints + actuationConstraints, nonNegativeDecisionVariables);
      bin.reshape(2 * staticEquilibriumConstraints + actuationConstraints, 1);
      costVectorC.reshape(nonNegativeDecisionVariables, 1);

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
      int actuationConstraintIndex = 0;
      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         ContactPointActuationConstraint actuationConstraint = input.getActuationConstraints().get(i);
         FrameVector3D contactNormal = input.getSurfaceNormals().get(i);

         if (actuationConstraint.isMaxNormalForceConstraint())
         {
            actuationConstraintCentroid.set(contactNormal);
            actuationConstraintCentroid.scale(actuationConstraint.getMaxNormalForce());
            addActuationConstraint(i, actuationConstraintIndex, actuationConstraintCentroid, contactNormal);
            actuationConstraintIndex++;
         }
         else
         {
            ConvexPolytope3D polytopeConstraint = actuationConstraint.getPolytopeConstraint();
            for (int j = 0; j < polytopeConstraint.getNumberOfFaces(); j++)
            {
               Face3D polytopeConstraintFace = polytopeConstraint.getFaces().get(j);
               addActuationConstraint(i, actuationConstraintIndex, polytopeConstraintFace.getCentroid(), polytopeConstraintFace.getNormal());
               actuationConstraintIndex++;
            }
         }
      }

      if (tickAndUpdatable != null)
      {
         averageContactPointPosition.setToZero();
         for (int i = 0; i < input.getNumberOfContacts(); i++)
         {
            averageContactPointPosition.add(input.getContactPointPositions().get(i));
         }
         averageContactPointPosition.scale(1.0 / input.getNumberOfContacts());
      }

      constructInitialApproximation();
   }

   private void addActuationConstraint(int contactPointIndex,
                                       int actuationConstraintIndex,
                                       Tuple3DReadOnly constraintPlanePoint,
                                       Tuple3DReadOnly constraintPlaneNormal)
   {
      int constraintRow = 2 * staticEquilibriumConstraints + actuationConstraintIndex;

      for (int i = 0; i < basisVectorsPerContactPoint; i++)
      {
         ContactPoint contactPoint = contactPoints.get(contactPointIndex);
         YoFrameVector3D basisVector = contactPoint.getBasisVector(i);
         Ain.set(constraintRow, basisVectorsPerContactPoint * contactPointIndex + i, basisVector.dot(constraintPlaneNormal));
      }

      bin.set(constraintRow, 0, constraintPlanePoint.dot(constraintPlaneNormal));
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

         boolean foundSolution = linearProgramSolver.solve(costVectorC, Ain, bin, solution);
         if (!foundSolution)
         {
            this.foundSolution.set(false);
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
         activeSetIndices.add(activeSet);

         updateGraphics();
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
               orderedActiveSetIndices.add(activeSetIndices.get(j));
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

   // new
   // compute an initial region by expanding along 0, 2pi/3, 4pi/3 degree directions
   private boolean constructInitialApproximation()
   {
      // multiplier denotes the evenly spaced initial directions
      final int multiplier = 3;
      final double incrementAngle = 2 * Math.PI / multiplier;
      for (int i = 0; i < multiplier; i++)
      {
         // unit vector in the desired direction
         double x = Math.cos(incrementAngle * i);
         double y = Math.sin(incrementAngle * i);

         // find the extremal point in the (x, y) direction
         costVectorC.set(nonNegativeDecisionVariables - 4, 0, x);
         costVectorC.set(nonNegativeDecisionVariables - 3, 0, y);
         costVectorC.set(nonNegativeDecisionVariables - 2, 0, -x);
         costVectorC.set(nonNegativeDecisionVariables - 1, 0, -y);

         if (linearProgramSolver.solve(costVectorC, Ain, bin, solution))
         {
            double x_sol = solution.get(nonNegativeDecisionVariables - 4) - solution.get(nonNegativeDecisionVariables - 2);
            double y_sol = solution.get(nonNegativeDecisionVariables - 3) - solution.get(nonNegativeDecisionVariables - 1);

            vertexList.add(new Point2D(x_sol, y_sol));
            supportRegion.addVertex(x_sol, y_sol);
            supportRegionVertices.add().setIncludingFrame(ReferenceFrame.getWorldFrame(), x_sol, y_sol, 0.0);
            supportRegion.update();

            updateGraphics();
         }
         else
         {
            LogTools.error("Solve iteration " + i + " failed");
            return false;
         }
      }

      return true;
   }

   //new
   public boolean solveBretl(int maxSolveIterations)
   {
      int n = 0;
      while (n < maxSolveIterations)
      {
         // make sure that the next vertex exists
         if (n < supportRegionVertices.size())
         {
            double x1 = supportRegionVertices.get(n).getX();
            double y1 = supportRegionVertices.get(n).getY();

            if (expandedSupportRegionVertices.contains(new FramePoint3D(ReferenceFrame.getWorldFrame(), x1, y1, 0)))
            {
               // skip if this vertex has already been expanded
               n++;
               continue;
            }

            int nextIndex = n + 1;
            if (nextIndex >= supportRegionVertices.size())
               nextIndex = 0;

            double x2 = supportRegionVertices.get(nextIndex).getX();
            double y2 = supportRegionVertices.get(nextIndex).getY();


            // compute orthogonal direction to the edge
            // (y2-y1, x1-x2) is normal to (x2-x1,y2-y1)
            double normalX = y2 - y1;
            double normalY = x1 - x2;

            // find the extremal point along computed normal
            costVectorC.set(nonNegativeDecisionVariables - 4, 0, normalX);
            costVectorC.set(nonNegativeDecisionVariables - 3, 0, normalY);
            costVectorC.set(nonNegativeDecisionVariables - 2, 0, -normalX);
            costVectorC.set(nonNegativeDecisionVariables - 1, 0, -normalY);

            if (linearProgramSolver.solve(costVectorC, Ain, bin, solution))
            {
               // x_sol and y_sol are the coordinates of the new vertex
               double x_sol = solution.get(nonNegativeDecisionVariables - 4) - solution.get(nonNegativeDecisionVariables - 2);
               double y_sol = solution.get(nonNegativeDecisionVariables - 3) - solution.get(nonNegativeDecisionVariables - 1);

               Point2D solVertex = new Point2D(x_sol, y_sol);

               //TODO: change this to only add the point if it's outside the current polygon
               if (!vertexList.contains(solVertex))
               {
                  vertexList.add(solVertex);
                  supportRegion.addVertex(x_sol, y_sol);
                  supportRegionVertices.insertAtIndex(nextIndex).setIncludingFrame(ReferenceFrame.getWorldFrame(), x_sol, y_sol, 0.0);
                  supportRegion.update();
                  updateGraphics();
               }

               //TODO: this will assume early convergence and can leave out some regions, make sure that all vertices have converged

               // check for convergence by computing area of triangle formed by (x_n, y_n), (x_n+1, y_n+1), (x_sol, y_sol)
               // this (signed) area is given by 0.5 * det([x_n - x_sol, y_n - y_sol; x_n+1 - x_sol, y_n+1 - y_sol])
               // if the area of this triangle is less than 1e-3, then add it to the list of expanded indices
               double area = 0.5 * (((x1 - x_sol) * (y2 - y_sol)) - ((y1 - y_sol) * (x2 - x_sol)));
               if ((Math.abs(area) < 1e-3) && (n > 3))
               {
                  expandedSupportRegionVertices.add().setIncludingFrame(ReferenceFrame.getWorldFrame(), x1, y1, 0.0);
               }

               n++;
            }
            else
            {
               LogTools.error("Solve iteration " + n + " failed");
               return false;
            }
         }
         else
         {
            // we ran out of vertices to optimize before reaching the max number of iterations
            LogTools.info("Finished in " + n + " iterations");
            break;
         }
      }

      return true;
   }

   public boolean foundSolution()
   {
      return foundSolution.getValue();
   }

   public List<TIntArrayList> getActiveSetIndices()
   {
      return orderedActiveSetIndices;
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
      activeSetIndices.clear();
      orderedActiveSetIndices.clear();
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
