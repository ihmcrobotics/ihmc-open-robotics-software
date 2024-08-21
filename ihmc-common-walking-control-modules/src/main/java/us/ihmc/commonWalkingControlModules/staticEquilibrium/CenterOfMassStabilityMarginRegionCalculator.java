package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.function.Supplier;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.ONE_TEN_MILLIONTH;

/**
 * Helper class for using {@link CenterOfMassStabilityMarginOptimizationModule} to update and manage a multi-contact stability region.
 */
public class CenterOfMassStabilityMarginRegionCalculator implements SCS2YoGraphicHolder
{
   private static final boolean DEBUG = false;
   private static final int NULL_MARGIN_INDEX = -1;

   // Hand-crafted vertex query order
   static final int[] VERTEX_ORDER = new int[]{0, 6, 12, 3, 9, 15, 1, 4, 7, 10, 13, 16, 2, 5, 8, 11, 14, 17};
   public static final int DIRECTIONS_TO_OPTIMIZE = VERTEX_ORDER.length;
   final static double DELTA_ANGLE = 2.0 * Math.PI / DIRECTIONS_TO_OPTIMIZE;
   private static final double[] QUERY_X = new double[DIRECTIONS_TO_OPTIMIZE];
   private static final double[] QUERY_Y = new double[DIRECTIONS_TO_OPTIMIZE];

   static
   {
      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         QUERY_X[i] = Math.cos(i * DELTA_ANGLE);
         QUERY_Y[i] = Math.sin(i * DELTA_ANGLE);
      }
   }

   private final String namePrefix;
   private ColorDefinition polygonGraphicColor = ColorDefinitions.Black();
   private ColorDefinition lowestMarginEdgeGraphicColor = ColorDefinitions.Red();

   /* Status variables */
   private final YoInteger queryCounter;
   private final YoBoolean hasSolvedWholeRegion;
   private final CenterOfMassStabilityMarginOptimizationModule optimizationModule;

   /* Solver solution data */
   private final DMatrixRMaj[] primalSolutions = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];
   private final DMatrixRMaj[] dualSolutions = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];

   /* CoM stability region */
   private final YoFramePoint2D[] optimizedVertices;
   private final YoFrameConvexPolygon2D feasibleCoMRegion;

   /* YoVariablized CoM for margin visualization */
   private final YoFramePoint2D yoCenterOfMass;

   /* Fields to monitor the nearest constraint edge */
   private final DMatrixRMaj[] resolvedForces = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];
   private final TIntArrayList[] saturatedConstraintIndices = new TIntArrayList[DIRECTIONS_TO_OPTIMIZE];
   private final TIntArrayList[] solutionBasisIndices = new TIntArrayList[DIRECTIONS_TO_OPTIMIZE];
   private final YoDouble[] minDictionaryRHSColumnEntries = new YoDouble[DIRECTIONS_TO_OPTIMIZE];
   private final YoFramePoint2D[] nearestConstraintVertexA = new YoFramePoint2D[DIRECTIONS_TO_OPTIMIZE];
   private final YoFramePoint2D[] nearestConstraintVertexB = new YoFramePoint2D[DIRECTIONS_TO_OPTIMIZE];
   private final YoInteger lowestMarginEdgeIndex;
   private final YoDouble centerOfMassStabilityMargin;

   /* Entry i corresponds to the distance of the CoM to the line segment connecting vertex (i) and (i+1) */
   private final YoDouble[] comEdgeMargin = new YoDouble[DIRECTIONS_TO_OPTIMIZE];

   private Supplier<FramePoint3DReadOnly> centerOfMassSupplier = null;
   private boolean showNearestSupportEdgeGraphic = true;

   public CenterOfMassStabilityMarginRegionCalculator(double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this("", robotMass, parentRegistry, graphicsListRegistry);
   }

   public CenterOfMassStabilityMarginRegionCalculator(String namePrefix, double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.namePrefix = namePrefix;
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      queryCounter = new YoInteger("queryIndex", registry);
      hasSolvedWholeRegion = new YoBoolean("hasSolvedWholeRegion", registry);

      optimizationModule = new CenterOfMassStabilityMarginOptimizationModule(robotMass, registry, graphicsListRegistry);
      optimizedVertices = new YoFramePoint2D[DIRECTIONS_TO_OPTIMIZE];

      for (int query_idx = 0; query_idx < DIRECTIONS_TO_OPTIMIZE; query_idx++)
      {
         optimizedVertices[query_idx] = new YoFramePoint2D("comStabilityMarginVertex" + query_idx, ReferenceFrame.getWorldFrame(), registry);
         resolvedForces[query_idx] = new DMatrixRMaj(0);
         saturatedConstraintIndices[query_idx] = new TIntArrayList();
         solutionBasisIndices[query_idx] = new TIntArrayList();
         minDictionaryRHSColumnEntries[query_idx] = new YoDouble("minDictionaryRHSColumnEntry" + query_idx, registry);
         comEdgeMargin[query_idx] = new YoDouble("comEdgeMargin" + query_idx, registry);
         nearestConstraintVertexA[query_idx] = new YoFramePoint2D("nearestConstraintVertexA_" + query_idx, ReferenceFrame.getWorldFrame(), registry);
         nearestConstraintVertexB[query_idx] = new YoFramePoint2D("nearestConstraintVertexB_" + query_idx, ReferenceFrame.getWorldFrame(), registry);

         minDictionaryRHSColumnEntries[query_idx].setToNaN();
         comEdgeMargin[query_idx].setToNaN();
         nearestConstraintVertexA[query_idx].setToNaN();
         nearestConstraintVertexB[query_idx].setToNaN();

         primalSolutions[query_idx] = new DMatrixRMaj(0);
         dualSolutions[query_idx] = new DMatrixRMaj(0);
      }

      feasibleCoMRegion = new YoFrameConvexPolygon2D("comStabilityMarginPolygon", ReferenceFrame.getWorldFrame(), DIRECTIONS_TO_OPTIMIZE, registry);
      feasibleCoMRegion.setToNaN();

      lowestMarginEdgeIndex = new YoInteger("lowestMarginEdgeIndex", registry);
      centerOfMassStabilityMargin = new YoDouble("centerOfMassStabilityMargin", registry);
      lowestMarginEdgeIndex.set(NULL_MARGIN_INDEX);
      centerOfMassStabilityMargin.set(Double.POSITIVE_INFINITY);

      yoCenterOfMass = new YoFramePoint2D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon multiContactCoMRegionArtifact = new YoArtifactPolygon("Multi-Contact CoM Region", feasibleCoMRegion, Color.BLACK, false);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), multiContactCoMRegionArtifact);

         YoArtifactPosition com = new YoArtifactPosition("CenterOfMass", yoCenterOfMass, GraphicType.SOLID_BALL, Color.ORANGE, 0.003);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), com);

         for (int query_idx = 0; query_idx < DIRECTIONS_TO_OPTIMIZE; query_idx++)
         {
            YoArtifactLineSegment2d nearestSegment = new YoArtifactLineSegment2d(namePrefix + "nearestConstraintViz" + query_idx, nearestConstraintVertexA[query_idx], nearestConstraintVertexB[query_idx], Color.RED);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), nearestSegment);

            YoArtifactPosition vertexGraphic = new YoArtifactPosition(namePrefix + "Point" + query_idx, optimizedVertices[query_idx], GraphicType.SOLID_BALL, Color.BLUE, 0.003);
            graphicsListRegistry.registerArtifact(getClass().getSimpleName(), vertexGraphic);
         }
      }
   }

   public void clear()
   {
      feasibleCoMRegion.clear();

      for (int query_idx = 0; query_idx < DIRECTIONS_TO_OPTIMIZE; query_idx++)
      {
         optimizedVertices[query_idx].setToNaN();
         resolvedForces[query_idx].zero();
         saturatedConstraintIndices[query_idx].reset();
         solutionBasisIndices[query_idx].reset();
         nearestConstraintVertexA[query_idx].setToNaN();
         nearestConstraintVertexB[query_idx].setToNaN();
      }

      hasSolvedWholeRegion.set(false);
      queryCounter.set(0);

      lowestMarginEdgeIndex.set(NULL_MARGIN_INDEX);
      centerOfMassStabilityMargin.set(Double.POSITIVE_INFINITY);
   }

   public void setupForStabilityMarginCalculation(Supplier<FramePoint3DReadOnly> centerOfMassSupplier)
   {
      this.centerOfMassSupplier = centerOfMassSupplier;
   }

   public void updateContactState(WholeBodyContactStateInterface contactState, boolean contactPointsHaveChanged)
   {
      optimizationModule.updateContactState(contactState, contactPointsHaveChanged);
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      optimizationModule.updateContactState(contactState);
   }

   public int getQueryCounter()
   {
      return queryCounter.getValue();
   }

   public boolean performFullRegionUpdate()
   {
      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         if (!performUpdateForVertex(i))
            return false;
      }
      for (int i = 0; i < DIRECTIONS_TO_OPTIMIZE; i++)
      {
         updateEdgeMargin(i);
      }
      updateMinimumMarginEdge();
      return true;
   }

   public boolean performUpdateForNextEdge()
   {
      int queryIndex = VERTEX_ORDER[queryCounter.getValue()];
      boolean success = performUpdateForEdge(queryIndex);
      if (success)
         queryCounter.set((queryCounter.getValue() + 1) % DIRECTIONS_TO_OPTIMIZE);
      return success;
   }

   public boolean performUpdateForEdge(int edgeIndex)
   {
      if (!performUpdateForVertex(getVertexAOfEdge(edgeIndex)))
         return false;
      if (!performUpdateForVertex(getVertexBOfEdge(edgeIndex)))
         return false;
      updateEdgeMargin(edgeIndex);
      updateMinimumMarginEdge();
      return true;
   }

   public void performFastUpdateForLowestMarginEdge(int edgeToTrust)
   {
      int vertexToTrustA = getVertexAOfEdge(edgeToTrust);
      int vertexToTrustB = getVertexBOfEdge(edgeToTrust);

      int vertexToUpdateA = getVertexAOfEdge(lowestMarginEdgeIndex.getValue());
      int vertexToUpdateB = getVertexBOfEdge(lowestMarginEdgeIndex.getValue());

      if (vertexToUpdateA != vertexToTrustA && vertexToUpdateA != vertexToTrustB)
         performFixedBasisUpdateForVertex(vertexToUpdateA);
      if (vertexToUpdateB != vertexToTrustA && vertexToUpdateB != vertexToTrustB)
         performFixedBasisUpdateForVertex(vertexToUpdateB);

      updateEdgeMargin(lowestMarginEdgeIndex.getValue());
      updateMinimumMarginEdge();
   }

   public boolean performUpdateForVertex(int vertexIndex)
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

         DMatrixRMaj optimizedForceAndCoM = optimizationModule.getOptimizedForceAndCoM();
         int numberOfForceVariables = optimizedForceAndCoM.getNumRows() - CenterOfMassStabilityMarginOptimizationModule.CoM_DIMENSIONS;
         if (resolvedForces[vertexIndex].getNumRows() != numberOfForceVariables)
         {
            resolvedForces[vertexIndex].reshape(numberOfForceVariables, 1);
         }

         for (int f_idx = 0; f_idx < numberOfForceVariables; f_idx++)
         {
            resolvedForces[vertexIndex].set(f_idx, optimizedForceAndCoM.get(f_idx, 0));
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

      optimizedVertices[vertexIndex].set(optimizationModule.getOptimizedCoM());

      feasibleCoMRegion.clear();
      boolean hasNaNVertex = false;
      for (int queryIdx = 0; queryIdx < DIRECTIONS_TO_OPTIMIZE; queryIdx++)
      {
         if (optimizedVertices[queryIdx].containsNaN())
            hasNaNVertex = true;
         else
            feasibleCoMRegion.addVertex(optimizedVertices[queryIdx]);
      }

      hasSolvedWholeRegion.set(!hasNaNVertex);
      feasibleCoMRegion.update();

      return true;
   }

   private void performFixedBasisUpdateForVertex(int vertexIndex)
   {
      optimizationModule.solveForFixedBasis(solutionBasisIndices[vertexIndex]);
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
   public CenterOfMassStabilityMarginOptimizationModule getOptimizationModule()
   {
      return optimizationModule;
   }

   private void updateEdgeMargin(int edgeIndex)
   {
      if (centerOfMassSupplier == null)
         return;

      YoFramePoint2D v0 = optimizedVertices[getVertexAOfEdge(edgeIndex)];
      YoFramePoint2D v1 = optimizedVertices[getVertexBOfEdge(edgeIndex)];

      if (v0.containsNaN() || v1.containsNaN() || v0.epsilonEquals(v1, 1.0e-3))
      {
         comEdgeMargin[edgeIndex].set(Double.NaN);
         return;
      }

      FramePoint3DReadOnly centerOfMass = centerOfMassSupplier.get();
      centerOfMass.getReferenceFrame().checkReferenceFrameMatch(v0.getReferenceFrame());

      double margin;
      if (v0.epsilonEquals(v1, 1.0e-5))
      {
         margin = centerOfMass.distanceXY(v0);
      }
      else
      {
         margin = EuclidGeometryTools.distanceFromPoint2DToLine2D(centerOfMass.getX(), centerOfMass.getY(), v0, v1);
      }
      comEdgeMargin[edgeIndex].set(margin);

      yoCenterOfMass.set(centerOfMass);
   }

   public void updateMinimumMarginEdge()
   {
      if (centerOfMassSupplier == null)
         return;

      double minimumMarginDistance = Double.POSITIVE_INFINITY;
      int minimumMarginEdgeIndex = NULL_MARGIN_INDEX;

      for (int edgeIdx = 0; edgeIdx < DIRECTIONS_TO_OPTIMIZE; edgeIdx++)
      {
         double margin_i = comEdgeMargin[edgeIdx].getValue();
         nearestConstraintVertexA[edgeIdx].setToNaN();
         nearestConstraintVertexB[edgeIdx].setToNaN();

         if (!Double.isNaN(margin_i) && margin_i < minimumMarginDistance)
         {
            minimumMarginDistance = margin_i;
            minimumMarginEdgeIndex = edgeIdx;
         }
      }

      if (minimumMarginEdgeIndex != NULL_MARGIN_INDEX)
      {
         centerOfMassStabilityMargin.set(minimumMarginDistance);
         lowestMarginEdgeIndex.set(minimumMarginEdgeIndex);

         nearestConstraintVertexA[minimumMarginEdgeIndex].set(optimizedVertices[getVertexAOfEdge(minimumMarginEdgeIndex)]);
         nearestConstraintVertexB[minimumMarginEdgeIndex].set(optimizedVertices[getVertexBOfEdge(minimumMarginEdgeIndex)]);
      }
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

   public DMatrixRMaj getResolvedForce(int query_idx)
   {
      return resolvedForces[query_idx];
   }

   public int getNumberOfVertices()
   {
      return DIRECTIONS_TO_OPTIMIZE;
   }

   public boolean hasSolvedWholeRegion()
   {
      return hasSolvedWholeRegion.getValue();
   }

   public FrameConvexPolygon2DReadOnly getFeasibleCoMRegion()
   {
      return feasibleCoMRegion;
   }

   public FramePoint2DReadOnly getOptimizedVertex(int index)
   {
      return optimizedVertices[index];
   }

   public FramePoint3DReadOnly getCenterOfMass()
   {
      return centerOfMassSupplier.get();
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

   public double getCenterOfMassStabilityMargin()
   {
      return centerOfMassStabilityMargin.getValue();
   }

   public boolean hasNearestConstraintEdge()
   {
      return lowestMarginEdgeIndex.getValue() != NULL_MARGIN_INDEX;
   }

   public int getLowestMarginEdgeIndex()
   {
      return lowestMarginEdgeIndex.getValue();
   }

   public DMatrixRMaj getSolverPrimalSolution(int queryIndex)
   {
      return primalSolutions[queryIndex];
   }

   public DMatrixRMaj getSolverDualSolution(int queryIndex)
   {
      return dualSolutions[queryIndex];
   }

   public TIntArrayList getSaturatedConstraintSet(int index)
   {
      return saturatedConstraintIndices[index];
   }

   public TIntArrayList getSolutionBasisIndices(int index)
   {
      return solutionBasisIndices[index];
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

         if (!Double.isNaN(comEdgeMargin[edgeIndex].getValue()) && comEdgeMargin[edgeIndex].getValue() <= centerOfMassStabilityMargin.getValue() + epsilon + ONE_TEN_MILLIONTH)
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D(namePrefix + "Multi-Contact CoM Region", feasibleCoMRegion, polygonGraphicColor, false));

      if (showNearestSupportEdgeGraphic)
      {
         for (int query_idx = 0; query_idx < DIRECTIONS_TO_OPTIMIZE; query_idx++)
         {
            YoGraphicLine2DDefinition nearestSegmentGraphic = YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition(namePrefix + "nearestConstraintViz" + query_idx,
                                                                                                                             nearestConstraintVertexA[query_idx],
                                                                                                                             nearestConstraintVertexB[query_idx],
                                                                                                                             lowestMarginEdgeGraphicColor);
            nearestSegmentGraphic.setStrokeWidth(1.0);
            group.addChild(nearestSegmentGraphic);

            YoGraphicPoint2DDefinition vertexGraphic = YoGraphicDefinitionFactory.newYoGraphicPoint2D(namePrefix + "Point" + query_idx,
                                                                                                      optimizedVertices[query_idx],
                                                                                                      0.003,
                                                                                                      ColorDefinitions.Blue(),
                                                                                                      DefaultPoint2DGraphic.CIRCLE_FILLED);
            group.addChild(vertexGraphic);
         }
      }

      if (CenterOfMassStabilityMarginOptimizationModule.DEBUG)
      {
         group.addChild(optimizationModule.getSCS2YoGraphics());
      }

      return group;
   }
}
