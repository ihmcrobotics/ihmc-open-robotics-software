package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicLine2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygon2DDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.Arrays;
import java.util.function.Supplier;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint2D;

/**
 * Helper class for using {@link StabilityMarginOptimizationModule} to update and manage a multi-contact stability region.
 */
public class StabilityMarginRegionCalculator implements SCS2YoGraphicHolder
{
   private static final int NULL_INDEX = -1;
   private static final double VERTEX_EPS = 1.0e-4;

   public static final int DIRECTIONS_TO_OPTIMIZE = 18;
   private final static double DELTA_ANGLE = 2.0 * Math.PI / DIRECTIONS_TO_OPTIMIZE;
   private static final double[] QUERY_X = new double[DIRECTIONS_TO_OPTIMIZE];
   private static final double[] QUERY_Y = new double[DIRECTIONS_TO_OPTIMIZE];

   static
   {
      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         QUERY_X[i] = Math.cos(i * DELTA_ANGLE);
         QUERY_Y[i] = -Math.sin(i * DELTA_ANGLE);
      }
   }

   private final String namePrefix;
   private ColorDefinition polygonGraphicColor;
   private ColorDefinition lowestMarginEdgeGraphicColor = ColorDefinitions.Red();

   /* Status variables */
   private final YoInteger queryCounter;
   private final YoBoolean hasSolvedWholeRegion;
   private final StabilityMarginOptimizationModule optimizationModule;

   /* Solver solution data */
   private final DMatrixRMaj[] primalSolutions = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];
   private final DMatrixRMaj[] dualSolutions = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];

   /* CoM stability region */
   private final YoFramePoint2D[] optimizedVertices = new YoFramePoint2D[DIRECTIONS_TO_OPTIMIZE];
   private final YoFrameConvexPolygon2D feasibleRegion;
   //   private final int[] toEuclidPolygonIndexMap = new int[DIRECTIONS_TO_OPTIMIZE];
   private final int[] fromEuclidPolygonIndexMap = new int[DIRECTIONS_TO_OPTIMIZE];

   /* YoVariablized CoM for margin visualization */
   private final YoFramePoint2D yoStabilityReference;
   private final YoFramePoint2D yoStabilityMarginPoint;

   /* Fields to monitor the nearest constraint edge */
   private final DMatrixRMaj[] resolvedForces = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];
   private final TIntArrayList[] saturatedConstraintIndices = new TIntArrayList[DIRECTIONS_TO_OPTIMIZE];
   private final TIntArrayList[] solutionBasisIndices = new TIntArrayList[DIRECTIONS_TO_OPTIMIZE];
   private final YoDouble[] minDictionaryRHSColumnEntries = new YoDouble[DIRECTIONS_TO_OPTIMIZE];
   private final YoFramePoint2D[] nearestConstraintVertexA = new YoFramePoint2D[DIRECTIONS_TO_OPTIMIZE];
   private final YoFramePoint2D[] nearestConstraintVertexB = new YoFramePoint2D[DIRECTIONS_TO_OPTIMIZE];
   private final YoInteger lowestMarginEdgeIndex;
   private final YoDouble stabilityMargin;

   /* Entry i corresponds to the distance of the CoM to the line segment connecting vertex (i) and (i+1) */
   private final YoDouble[] comEdgeMargin = new YoDouble[DIRECTIONS_TO_OPTIMIZE];

   private Supplier<FramePoint3DReadOnly> stabilityReference = null;
   private boolean showNearestSupportEdgeGraphic = true;

   public StabilityMarginRegionCalculator(StabilityMarginOptimizationModule optimizationModule, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this("", optimizationModule, parentRegistry, graphicsListRegistry);
   }

   public StabilityMarginRegionCalculator(String namePrefix, StabilityMarginOptimizationModule optimizationModule, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.namePrefix = namePrefix;
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      queryCounter = new YoInteger("queryIndex", registry);
      hasSolvedWholeRegion = new YoBoolean("hasSolvedWholeRegion", registry);
      this.optimizationModule = optimizationModule;
      polygonGraphicColor = optimizationModule.getRegionGraphicColor();

      for (int vertex_idx = 0; vertex_idx < DIRECTIONS_TO_OPTIMIZE; vertex_idx++)
      {
         optimizedVertices[vertex_idx] = new YoFramePoint2D("comStabilityMarginVertex" + vertex_idx, ReferenceFrame.getWorldFrame(), registry);
         resolvedForces[vertex_idx] = new DMatrixRMaj(0);
         saturatedConstraintIndices[vertex_idx] = new TIntArrayList();
         solutionBasisIndices[vertex_idx] = new TIntArrayList();
         minDictionaryRHSColumnEntries[vertex_idx] = new YoDouble("minDictionaryRHSColumnEntry" + vertex_idx, registry);
         comEdgeMargin[vertex_idx] = new YoDouble("comEdgeMargin" + vertex_idx, registry);
         nearestConstraintVertexA[vertex_idx] = new YoFramePoint2D("nearestConstraintVertexA_" + vertex_idx, ReferenceFrame.getWorldFrame(), registry);
         nearestConstraintVertexB[vertex_idx] = new YoFramePoint2D("nearestConstraintVertexB_" + vertex_idx, ReferenceFrame.getWorldFrame(), registry);

         minDictionaryRHSColumnEntries[vertex_idx].setToNaN();
         comEdgeMargin[vertex_idx].setToNaN();
         nearestConstraintVertexA[vertex_idx].setToNaN();
         nearestConstraintVertexB[vertex_idx].setToNaN();

         primalSolutions[vertex_idx] = new DMatrixRMaj(0);
         dualSolutions[vertex_idx] = new DMatrixRMaj(0);
      }

      feasibleRegion = new YoFrameConvexPolygon2D(namePrefix + "StabilityMarginPolygon", ReferenceFrame.getWorldFrame(), DIRECTIONS_TO_OPTIMIZE, registry);

      lowestMarginEdgeIndex = new YoInteger("lowestMarginEdgeIndex", registry);
      stabilityMargin = new YoDouble(namePrefix + "StabilityMargin", registry);
      lowestMarginEdgeIndex.set(NULL_INDEX);
      stabilityMargin.set(Double.POSITIVE_INFINITY);

      yoStabilityReference = new YoFramePoint2D("stabilityReference", ReferenceFrame.getWorldFrame(), registry);
      yoStabilityMarginPoint = new YoFramePoint2D("stabilityMarginPoint", ReferenceFrame.getWorldFrame(), registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon multiContactCoMRegionArtifact = new YoArtifactPolygon(namePrefix + " Multi-Contact Region", feasibleRegion, Color.BLACK, false, 5);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), multiContactCoMRegionArtifact);

         YoArtifactPosition stabilityReference = new YoArtifactPosition(namePrefix + "StabilityReference", yoStabilityReference, GraphicType.BALL_WITH_CROSS, Color.BLACK, 0.01);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), stabilityReference);

         for (int vertex_idx = 0; vertex_idx < DIRECTIONS_TO_OPTIMIZE; vertex_idx++)
         {
            YoArtifactLineSegment2d nearestSegment = new YoArtifactLineSegment2d(namePrefix + "nearestConstraintViz" + vertex_idx, nearestConstraintVertexA[vertex_idx], nearestConstraintVertexB[vertex_idx], Color.RED);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), nearestSegment);

            YoArtifactPosition vertexGraphic = new YoArtifactPosition(namePrefix + "Point" + vertex_idx, optimizedVertices[vertex_idx], GraphicType.SOLID_BALL, Color.BLUE, 0.003);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), vertexGraphic);
         }
      }
   }

   public static StabilityMarginRegionCalculator createForCoMStabilityMargin(String prefix, double robotMass, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      CenterOfMassStabilityMarginOptimizationModule stabilityMarginOptimizationModule = new CenterOfMassStabilityMarginOptimizationModule(prefix,
                                                                                                                                          robotMass,
                                                                                                                                          registry,
                                                                                                                                          graphicsListRegistry);
      return new StabilityMarginRegionCalculator(prefix, stabilityMarginOptimizationModule, registry, graphicsListRegistry);
   }

   public static StabilityMarginRegionCalculator createForCoPStabilityMargin(String prefix,
                                                                             double robotMass,
                                                                             ReferenceFrame centerOfMassFrame,
                                                                             ReferenceFrame midFeetZUpFrame,
                                                                             YoRegistry registry,
                                                                             YoGraphicsListRegistry graphicsListRegistry)
   {
      CenterOfPressureStabilityMarginOptimizationModule stabilityMarginOptimizationModule = new CenterOfPressureStabilityMarginOptimizationModule(prefix,
                                                                                                                                                  robotMass,
                                                                                                                                                  centerOfMassFrame,
                                                                                                                                                  midFeetZUpFrame,
                                                                                                                                                  registry,
                                                                                                                                                  graphicsListRegistry);
      return new StabilityMarginRegionCalculator(prefix, stabilityMarginOptimizationModule, registry, graphicsListRegistry);
   }

   public void clear()
   {
      feasibleRegion.clear();

      for (int vertex_idx = 0; vertex_idx < DIRECTIONS_TO_OPTIMIZE; vertex_idx++)
      {
         optimizedVertices[vertex_idx].setToNaN();
         resolvedForces[vertex_idx].zero();
         saturatedConstraintIndices[vertex_idx].reset();
         solutionBasisIndices[vertex_idx].reset();
         nearestConstraintVertexA[vertex_idx].setToNaN();
         nearestConstraintVertexB[vertex_idx].setToNaN();
      }

      hasSolvedWholeRegion.set(false);
      queryCounter.set(0);

      lowestMarginEdgeIndex.set(NULL_INDEX);
      stabilityMargin.set(Double.POSITIVE_INFINITY);
   }

   public void setupForStabilityMarginCalculation(Supplier<FramePoint3DReadOnly> stabilityReference)
   {
      this.stabilityReference = stabilityReference;
   }

   public void updateContactState(WholeBodyContactStateInterface contactState, boolean contactPointsHaveChanged)
   {
      optimizationModule.updateContactState(contactState, contactPointsHaveChanged);
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      optimizationModule.updateContactState(contactState);
   }

   public boolean performFullRegionUpdate()
   {
      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         if (!performUpdateForVertex(i))
            return false;
      }

      updateFeasibleRegion();
      updateMinimumMarginEdge();

      return true;
   }

   public boolean performUpdateForNextVertex()
   {
      int vertexIndexToUpdate = queryCounter.getValue();
      boolean success = performUpdateForVertex(vertexIndexToUpdate);
      if (success)
         queryCounter.set((queryCounter.getValue() + 1) % DIRECTIONS_TO_OPTIMIZE);
      updateFeasibleRegion();
      return success;
   }

   private boolean performUpdateForVertex(int vertexIndex)
   {
      double queryX = queryDirectionX(vertexIndex);
      double queryY = queryDirectionY(vertexIndex);
      boolean success = optimizationModule.solve(queryX, queryY);

      if (success)
      {
         saturatedConstraintIndices[vertexIndex].reset();
         solutionBasisIndices[vertexIndex].reset();

         solutionBasisIndices[vertexIndex].addAll(optimizationModule.getLinearProgramSolver().getBasisIndices());
         TIntArrayList nonBasisIndices = optimizationModule.getLinearProgramSolver().getNonBasisIndices();
         for (int i = 1; i < nonBasisIndices.size(); i++)
         {
            int lexicalIndex = nonBasisIndices.get(i);
            if (!optimizationModule.getLinearProgramSolver().isNonNegativeConstraint(lexicalIndex))
            {
               saturatedConstraintIndices[vertexIndex].add(optimizationModule.getLinearProgramSolver().toConstraintIndex(lexicalIndex));
            }
         }

         DMatrixRMaj solution = optimizationModule.getNominalSolution();
         int numberOfForceVariables = optimizationModule.getNumberOfNominalVariables();
         if (resolvedForces[vertexIndex].getNumRows() != numberOfForceVariables)
         {
            resolvedForces[vertexIndex].reshape(numberOfForceVariables, 1);
         }

         for (int f_idx = 0; f_idx < numberOfForceVariables; f_idx++)
         {
            resolvedForces[vertexIndex].set(f_idx, solution.get(f_idx, 0));
         }

         minDictionaryRHSColumnEntries[vertexIndex].set(optimizationModule.getLinearProgramSolver().getSimplexStatistics().getMinDictionaryRHSColumnEntry());
         primalSolutions[vertexIndex].set(optimizationModule.getSolverSolution());
         dualSolutions[vertexIndex].set(optimizationModule.getLinearProgramSolver().getDualSolution());
      }
      else
      {
         optimizedVertices[vertexIndex].setToNaN();
         resolvedForces[vertexIndex].zero();
         saturatedConstraintIndices[vertexIndex].reset();
         solutionBasisIndices[vertexIndex].reset();
         hasSolvedWholeRegion.set(false);
         comEdgeMargin[vertexIndex].setToNaN();
         minDictionaryRHSColumnEntries[vertexIndex].setToNaN();
         CommonOps_DDRM.fill(primalSolutions[vertexIndex], Double.NaN);
         CommonOps_DDRM.fill(dualSolutions[vertexIndex], Double.NaN);
         return false;
      }

      Point2DReadOnly optimizedStabilityPoint = optimizationModule.getOptimizedStabilityPoint();
      optimizedVertices[vertexIndex].set(optimizedStabilityPoint);

      return true;
   }

   private void updateFeasibleRegion()
   {
      // Update ConvexPolygon2D object

      feasibleRegion.clear();
      boolean hasNaNVertex = false;
      for (int vertex_idx = 0; vertex_idx < DIRECTIONS_TO_OPTIMIZE; vertex_idx++)
      {
         if (optimizedVertices[vertex_idx].containsNaN())
            hasNaNVertex = true;
         else
            feasibleRegion.addVertex(optimizedVertices[vertex_idx]);
      }

      feasibleRegion.update();
      hasSolvedWholeRegion.set(!hasNaNVertex);

      if (!hasSolvedWholeRegion.getValue())
         return;

      // Update index correspondence map
//      Arrays.fill(fromEuclidPolygonIndexMap, NULL_INDEX);
//
//      for (int euclid_idx = 0; euclid_idx < feasibleRegion.getNumberOfVertices(); euclid_idx++)
//      {
//         int enumerated_idx = findVertexIndex(optimizedVertices, feasibleRegion.getVertex(euclid_idx));
//
//         if (enumerated_idx != NULL_INDEX)
//         {
//            fromEuclidPolygonIndexMap[euclid_idx] = enumerated_idx;
//         }
//      }

      updateMinimumMarginEdge();
   }

   private static int findVertexIndex(Tuple2DReadOnly[] candidateVertices, Tuple2DReadOnly vertex)
   {
      for (int vertex_idx = 0; vertex_idx < candidateVertices.length; vertex_idx++)
      {
         if (vertex.epsilonEquals(candidateVertices[vertex_idx], VERTEX_EPS))
            return vertex_idx;
      }
      return NULL_INDEX;
   }

   /**
    * Adapted from EuclidGeometryPolygonTools#closestEdgeIndexToPoint2D
    */
   private void updateMinimumMarginEdge()
   {
      FramePoint3DReadOnly stabilityReference = this.stabilityReference.get();
      if (stabilityReference == null)
         return;

      double stabilityReferenceX = stabilityReference.getX();
      double stabilityReferenceY = stabilityReference.getY();
      yoStabilityReference.set(stabilityReferenceX, stabilityReferenceY);

      boolean isQueryOutsidePolygon = false;
      int insideIndex = -1;
      int outsideIndex = -1;
      double minOutsideDistanceSquared = Double.POSITIVE_INFINITY;
      double minInsideDistanceSquared = Double.POSITIVE_INFINITY;

      for (int edgeIndex = 0; edgeIndex < feasibleRegion.getNumberOfVertices(); edgeIndex++)
      {
         Point2DReadOnly edgeStart = feasibleRegion.getVertex(edgeIndex);
         Point2DReadOnly edgeEnd = feasibleRegion.getNextVertex(edgeIndex);

         double distanceSquared = distanceSquaredFromPoint2DToLineSegment2D(stabilityReferenceX, stabilityReferenceY, edgeStart, edgeEnd);

         boolean isOutsideEdge = isPoint2DOnSideOfLine2D(stabilityReferenceX, stabilityReferenceY, edgeStart, edgeEnd, feasibleRegion.isClockwiseOrdered());
         isQueryOutsidePolygon |= isOutsideEdge;

         /*
          * By keeping track of whether the point is outside or inside w.r.t. to each edge, it is ensured
          * that if the point is outside the polygon the edge returned is visible from the query.
          */
         if (isOutsideEdge)
         {
            if (distanceSquared < minOutsideDistanceSquared)
            {
               outsideIndex = edgeIndex;
               minOutsideDistanceSquared = distanceSquared;
            }
         }
         else
         {
            if (distanceSquared < minInsideDistanceSquared)
            {
               insideIndex = edgeIndex;
               minInsideDistanceSquared = distanceSquared;
            }
         }
      }

      lowestMarginEdgeIndex.set(isQueryOutsidePolygon ? outsideIndex : insideIndex);
      stabilityMargin.set(Math.sqrt(isQueryOutsidePolygon ? minOutsideDistanceSquared : minInsideDistanceSquared));

      FramePoint2DReadOnly v1 = feasibleRegion.getVertex(lowestMarginEdgeIndex.getValue());
      FramePoint2DReadOnly v2 = feasibleRegion.getNextVertex(lowestMarginEdgeIndex.getValue());
      EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(stabilityReferenceX, stabilityReferenceY, v1.getX(), v1.getY(), v2.getX(), v2.getY(), yoStabilityMarginPoint);

      for (int vertex_idx = 0; vertex_idx < DIRECTIONS_TO_OPTIMIZE; vertex_idx++)
      {
         nearestConstraintVertexA[vertex_idx].setToNaN();
         nearestConstraintVertexB[vertex_idx].setToNaN();
      }

      nearestConstraintVertexA[lowestMarginEdgeIndex.getValue()].set(feasibleRegion.getVertex(lowestMarginEdgeIndex.getValue()));
      nearestConstraintVertexB[lowestMarginEdgeIndex.getValue()].set(feasibleRegion.getNextVertex(lowestMarginEdgeIndex.getValue()));
   }

   private void performFixedBasisUpdateForVertex(int vertexIndex)
   {
      optimizedVertices[vertexIndex].set(optimizationModule.solveForFixedBasis(solutionBasisIndices[vertexIndex]));
   }

   public static double queryDirectionX(int index)
   {
      return QUERY_X[index];
   }

   public static double queryDirectionY(int index)
   {
      return QUERY_Y[index];
   }

   // TODO should make package private later?
   public StabilityMarginOptimizationModule getOptimizationModule()
   {
      return optimizationModule;
   }

   /**
    * Edge i corresponds to vertices (i) and (i + 1), denoted vertices A and B of the edge
    */
   /* package-private */ static int getVertexAOfEdge(int edgeIndex)
   {
      return edgeIndex;
   }

   /**
    * Edge i corresponds to vertices (i) and (i + 1), denoted vertices A and B of the edge
    */
   /* package-private */ static int getVertexBOfEdge(int edgeIndex)
   {
      return getNextIndex(edgeIndex);
   }

   /**
    * Vertex i corresponds to edges (i - 1) and (i), denoted edge A and B of the vertex
    */
   /* package-private */ static int getEdgeAOfVertex(int vertexIndex)
   {
      return getPreviousIndex(vertexIndex);
   }

   /**
    * Vertex i corresponds to edges (i - 1) and (i), denoted edge A and B of the vertex
    */
   /* package-private */ static int getEdgeBOfVertex(int vertexIndex)
   {
      return vertexIndex;
   }

   private static int getPreviousIndex(int index)
   {
      return index > 0 ? index - 1 : DIRECTIONS_TO_OPTIMIZE - 1;
   }

   private static int getNextIndex(int index)
   {
      return (index + 1) % DIRECTIONS_TO_OPTIMIZE;
   }

   public DMatrixRMaj getResolvedForce(int vertex_idx)
   {
      return resolvedForces[vertex_idx];
   }

   public int getNumberOfVertices()
   {
      return DIRECTIONS_TO_OPTIMIZE;
   }

   public boolean hasSolvedWholeRegion()
   {
      return hasSolvedWholeRegion.getValue();
   }

   public FrameConvexPolygon2DReadOnly getFeasibleRegion()
   {
      return feasibleRegion;
   }

   public FramePoint2DReadOnly getOptimizedVertex(int index)
   {
      return optimizedVertices[index];
   }

   public int fromEuclidIndex(int euclidIndex)
   {
      return fromEuclidPolygonIndexMap[euclidIndex];
   }

   public FramePoint3DReadOnly getCenterOfMass()
   {
      return stabilityReference.get();
   }

   public LinearProgramSolver getSolver()
   {
      return optimizationModule.getLinearProgramSolver();
   }

   public void setGraphicColors(ColorDefinition polygonGraphicColor, ColorDefinition lowestMarginEdgeGraphicColor)
   {
      this.polygonGraphicColor = polygonGraphicColor;
      this.lowestMarginEdgeGraphicColor = lowestMarginEdgeGraphicColor;
   }

   public double getStabilityMargin()
   {
      return stabilityMargin.getValue();
   }

   public boolean hasNearestConstraintEdge()
   {
      return lowestMarginEdgeIndex.getValue() != NULL_INDEX;
   }

   public int getLowestMarginEdgeIndex()
   {
      return lowestMarginEdgeIndex.getValue();
   }

   public DMatrixRMaj getSolverPrimalSolution(int vertexIndex)
   {
      return primalSolutions[vertexIndex];
   }

   public DMatrixRMaj getSolverDualSolution(int vertexIndex)
   {
      return dualSolutions[vertexIndex];
   }

   public TIntArrayList getSaturatedConstraintSet(int vertexIndex)
   {
      return saturatedConstraintIndices[vertexIndex];
   }

   public TIntArrayList getSolutionBasisIndices(int vertexIndex)
   {
      return solutionBasisIndices[vertexIndex];
   }

   public int collectLowestMarginVertexIndices(TIntArrayList lowestMarginVertexIndices, double epsilon)
   {
      lowestMarginVertexIndices.reset();

      if (!hasNearestConstraintEdge())
         return 0;

      int numberOfLowMarginEdges = 0;

      for (int edgeIndex = 0; edgeIndex < DIRECTIONS_TO_OPTIMIZE; edgeIndex++)
      {
         int indexA = getVertexAOfEdge(edgeIndex);
         int indexB = getVertexBOfEdge(edgeIndex);

         if (!Double.isNaN(comEdgeMargin[edgeIndex].getValue()) && comEdgeMargin[edgeIndex].getValue() <= stabilityMargin.getValue() + epsilon + ONE_TEN_MILLIONTH)
         {
            if (!lowestMarginVertexIndices.contains(indexA))
               lowestMarginVertexIndices.add(indexA);
            if (!lowestMarginVertexIndices.contains(indexB))
               lowestMarginVertexIndices.add(indexB);
            nearestConstraintVertexA[edgeIndex].set(optimizedVertices[indexA]);
            nearestConstraintVertexB[edgeIndex].set(optimizedVertices[indexB]);
            numberOfLowMarginEdges++;
         }
         else
         {
            nearestConstraintVertexA[edgeIndex].setToNaN();
            nearestConstraintVertexB[edgeIndex].setToNaN();
         }
      }

      return numberOfLowMarginEdges;
   }

   public void setShowNearestSupportEdgeGraphic(boolean showNearestSupportEdgeGraphic)
   {
      this.showNearestSupportEdgeGraphic = showNearestSupportEdgeGraphic;
   }

   private final FramePoint2D tempPointA = new FramePoint2D();
   private final FramePoint2D tempPointB = new FramePoint2D();

   public void initializeRegion(ReferenceFrame midFeetZUpFrame, RecyclingArrayList<Point2D> vertices)
   {
      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         double queryX = QUERY_X[i];
         double queryY = QUERY_Y[i];

         double maxValue = Double.NEGATIVE_INFINITY;
         for (int j = 0; j < vertices.size(); j++)
         {
            tempPointA.setIncludingFrame(midFeetZUpFrame, vertices.get(j));
            tempPointA.changeFrame(ReferenceFrame.getWorldFrame());
            double value = tempPointA.getX() * queryX + tempPointA.getY() * queryY;
            if (value > maxValue)
            {
               maxValue = value;
               tempPointB.setIncludingFrame(tempPointA);
            }
         }

         optimizedVertices[i].set(tempPointB);
      }

      updateFeasibleRegion();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      YoGraphicPolygon2DDefinition regionGraphic = YoGraphicDefinitionFactory.newYoGraphicPolygon2D(namePrefix + "Multi-Contact CoM Region", feasibleRegion,
                                                                                                    polygonGraphicColor,
                                                                                                    false);
      regionGraphic.setStrokeWidth(1.5);
      group.addChild(regionGraphic);

      if (showNearestSupportEdgeGraphic)
      {
         for (int vertex_idx = 0; vertex_idx < DIRECTIONS_TO_OPTIMIZE; vertex_idx++)
         {
            YoGraphicLine2DDefinition nearestSegmentGraphic = YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition(namePrefix + "nearestConstraintViz" + vertex_idx,
                                                                                                                             nearestConstraintVertexA[vertex_idx],
                                                                                                                             nearestConstraintVertexB[vertex_idx],
                                                                                                                             lowestMarginEdgeGraphicColor);
            nearestSegmentGraphic.setStrokeWidth(1.0);
            group.addChild(nearestSegmentGraphic);

            YoGraphicPoint2DDefinition vertexGraphic = newYoGraphicPoint2D(namePrefix + "Point" + vertex_idx,
                                                                           optimizedVertices[vertex_idx],
                                                                           0.003,
                                                                           ColorDefinitions.Blue(),
                                                                           DefaultPoint2DGraphic.CIRCLE_FILLED);
            group.addChild(vertexGraphic);
         }
      }

      YoGraphicPoint2DDefinition vertexGraphic = newYoGraphicPoint2D(namePrefix + "StabilityMarginPoint",
                                                                                                yoStabilityMarginPoint,
                                                                                                0.003,
                                                                                                ColorDefinitions.Blue(),
                                                                                                DefaultPoint2DGraphic.CIRCLE_FILLED);
      group.addChild(vertexGraphic);

      if (StabilityMarginOptimizationModule.DEBUG)
      {
         group.addChild(optimizationModule.getSCS2YoGraphics());
      }

      return group;
   }
}
