package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.convexOptimization.linearProgram.LinearProgramSolver;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicLine2DDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;
import java.util.function.Supplier;

/**
 * Helper class for using {@link CenterOfMassStabilityMarginOptimizationModule} to update and manage a multi-contact stability region.
 */
public class CenterOfMassStabilityMarginRegionCalculator implements SCS2YoGraphicHolder
{
   private static final boolean DEBUG = false;
   private static final int NULL_MARGIN_INDEX = -1;

   // Hand-crafted vertex query order
   private static final int[] VERTEX_ORDER = new int[]{0, 6, 12, 3, 9, 15, 1, 4, 7, 10, 13, 16, 2, 5, 8, 11, 14, 17};
   private static final int DIRECTIONS_TO_OPTIMIZE = VERTEX_ORDER.length;

   private final String namePrefix;
   private ColorDefinition polygonGraphicColor = ColorDefinitions.Black();
   private ColorDefinition lowestMarginEdgeGraphicColor = ColorDefinitions.Red();

   /* Status variables */
   private final double deltaAngle = 2.0 * Math.PI / DIRECTIONS_TO_OPTIMIZE;
   private final YoInteger queryCounter;
   private final YoBoolean hasSolvedWholeRegion;
   private final CenterOfMassStabilityMarginOptimizationModule optimizationModule;

   /* CoM stability region */
   private final YoFramePoint2D[] optimizedVertices;
   private final YoFrameConvexPolygon2D feasibleCoMRegion;

   /* Fields to monitor nearest constraint edge */
   private final DMatrixRMaj[] resolvedForces = new DMatrixRMaj[DIRECTIONS_TO_OPTIMIZE];
   private final TIntArrayList[] saturatedConstraintIndices = new TIntArrayList[DIRECTIONS_TO_OPTIMIZE];
   private final YoDouble[] minDictionaryRHSColumnEntries = new YoDouble[DIRECTIONS_TO_OPTIMIZE];
   private final YoFramePoint2D nearestConstraintVertexA;
   private final YoFramePoint2D nearestConstraintVertexB;
   private final YoInteger lowestMarginEdgeIndex;
   private final YoDouble centerOfMassStabilityMargin;

   /* Entry i corresponds to the distance of the CoM to the line segment connecting vertex (i) and (i+1) */
   private final YoDouble[] comEdgeMargin = new YoDouble[DIRECTIONS_TO_OPTIMIZE];

   private Supplier<FramePoint3DReadOnly> centerOfMassSupplier = null;
   private boolean showNearestSupportEdgeGraphic = true;

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
         minDictionaryRHSColumnEntries[query_idx] = new YoDouble("minDictionaryRHSColumnEntry" + query_idx, registry);
         comEdgeMargin[query_idx] = new YoDouble("comEdgeMargin" + query_idx, registry);

         minDictionaryRHSColumnEntries[query_idx].setToNaN();
         comEdgeMargin[query_idx].setToNaN();
      }

      feasibleCoMRegion = new YoFrameConvexPolygon2D("comStabilityMarginPolygon", ReferenceFrame.getWorldFrame(), DIRECTIONS_TO_OPTIMIZE, registry);
      feasibleCoMRegion.setToNaN();

      nearestConstraintVertexA = new YoFramePoint2D("nearestConstraintVertexA", ReferenceFrame.getWorldFrame(), registry);
      nearestConstraintVertexB = new YoFramePoint2D("nearestConstraintVertexB", ReferenceFrame.getWorldFrame(), registry);
      lowestMarginEdgeIndex = new YoInteger("lowestMarginEdgeIndex", registry);
      centerOfMassStabilityMargin = new YoDouble("centerOfMassStabilityMargin", registry);
      lowestMarginEdgeIndex.set(NULL_MARGIN_INDEX);
      centerOfMassStabilityMargin.set(Double.POSITIVE_INFINITY);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
      {
         YoArtifactPolygon multiContactCoMRegionArtifact = new YoArtifactPolygon("Multi-Contact CoM Region", feasibleCoMRegion, Color.BLACK, false);
         graphicsListRegistry.registerArtifact(getClass().getSimpleName(), multiContactCoMRegionArtifact);
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
      }

      hasSolvedWholeRegion.set(false);
      queryCounter.set(0);

      nearestConstraintVertexA.setToNaN();
      nearestConstraintVertexB.setToNaN();
      lowestMarginEdgeIndex.set(NULL_MARGIN_INDEX);
      centerOfMassStabilityMargin.set(Double.POSITIVE_INFINITY);
   }

   public void setupForStabilityMarginCalculation(Supplier<FramePoint3DReadOnly> centerOfMassSupplier)
   {
      this.centerOfMassSupplier = centerOfMassSupplier;
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      optimizationModule.updateContactState(contactState);
   }

   public int getQueryCounter()
   {
      return queryCounter.getValue();
   }

   public boolean performCoMRegionQuery()
   {
      int queryIndex = VERTEX_ORDER[queryCounter.getValue()];
      boolean success = performCoMRegionQuery(queryIndex);
      if (success)
         queryCounter.set((queryCounter.getValue() + 1) % DIRECTIONS_TO_OPTIMIZE);
      return success;
   }

   public boolean performCoMRegionQuery(int queryIndex)
   {
      double queryX = Math.cos(queryIndex * deltaAngle);
      double queryY = Math.sin(queryIndex * deltaAngle);
      boolean success = optimizationModule.solve(queryX, queryY);

      if (success)
      {
         saturatedConstraintIndices[queryIndex].reset();

         TIntArrayList nonBasisIndices = null;
         for (int i = 1; i < nonBasisIndices.size(); i++)
         {
            int lexicalIndex = nonBasisIndices.get(i);
            if (true)
            {
//               saturatedConstraintIndices[queryIndex].add(optimizationModule.getLinearProgramSolver().toConstraintIndex(lexicalIndex));
            }
         }

         DMatrixRMaj optimizedForceAndCoM = optimizationModule.getOptimizedForceAndCoM();
         int numberOfForceVariables = optimizedForceAndCoM.getNumRows() - CenterOfMassStabilityMarginOptimizationModule.CoM_DIMENSIONS;
         if (resolvedForces[queryIndex].getNumRows() != numberOfForceVariables)
         {
            resolvedForces[queryIndex].reshape(numberOfForceVariables, 1);
         }

         for (int f_idx = 0; f_idx < numberOfForceVariables; f_idx++)
         {
            resolvedForces[queryIndex].set(f_idx, optimizedForceAndCoM.get(f_idx, 0));
         }

//         minDictionaryRHSColumnEntries[queryIndex].set(optimizationModule.getLinearProgramSolver().getSimplexStatistics().getMinDictionaryRHSColumnEntry());
      }
      else
      {
         optimizedVertices[queryIndex].setToNaN();
         resolvedForces[queryIndex].zero();
         saturatedConstraintIndices[queryIndex].reset();
         hasSolvedWholeRegion.set(false);
         comEdgeMargin[queryIndex].setToNaN();
         minDictionaryRHSColumnEntries[queryIndex].setToNaN();
         return false;
      }

      optimizedVertices[queryIndex].set(optimizationModule.getOptimizedCoM());

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

      if (centerOfMassSupplier != null)
      {
         updateNearestVertex(getEdgeAOfVertex(queryIndex));
         updateNearestVertex(getEdgeBOfVertex(queryIndex));
      }

      return true;
   }

   private void updateNearestVertex(int edgeIndex)
   {
      YoFramePoint2D v0 = optimizedVertices[getVertexAOfEdge(edgeIndex)];
      YoFramePoint2D v1 = optimizedVertices[getVertexBOfEdge(edgeIndex)];

      if (v0.containsNaN() || v1.containsNaN())
      {
         comEdgeMargin[edgeIndex].set(Double.NaN);
         return;
      }

      FramePoint3DReadOnly centerOfMass = centerOfMassSupplier.get();
      centerOfMass.getReferenceFrame().checkReferenceFrameMatch(v0.getReferenceFrame());

      double margin = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(centerOfMass.getX(), centerOfMass.getY(), v0, v1);
      comEdgeMargin[edgeIndex].set(margin);

      double minimumMarginDistance = Double.POSITIVE_INFINITY;
      int minimumMarginEdgeIndex = NULL_MARGIN_INDEX;

      for (int edgeIdx = 0; edgeIdx < DIRECTIONS_TO_OPTIMIZE; edgeIdx++)
      {
         double margin_i = comEdgeMargin[edgeIdx].getValue();
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
         nearestConstraintVertexA.set(optimizedVertices[getVertexAOfEdge(minimumMarginEdgeIndex)]);
         nearestConstraintVertexB.set(optimizedVertices[getVertexBOfEdge(minimumMarginEdgeIndex)]);
      }
   }

   private static int getVertexAOfEdge(int edgeIndex)
   {
      return edgeIndex;
   }

   private static int getVertexBOfEdge(int edgeIndex)
   {
      return getNextIndex(edgeIndex);
   }

   private static int getEdgeAOfVertex(int vertexIndex)
   {
      return getPreviousIndex(vertexIndex);
   }

   private static int getEdgeBOfVertex(int vertexIndex)
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

   public YoFramePoint2D getOptimizedVertex(int index)
   {
      return optimizedVertices[index];
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

   private boolean hasNearestConstraintEdge()
   {
      return lowestMarginEdgeIndex.getValue() != NULL_MARGIN_INDEX;
   }

   public TIntArrayList getSaturatedConstraintSet(int index)
   {
      return saturatedConstraintIndices[index];
   }

   public int collectLowestMarginIndices(TIntArrayList lowestMarginIndices, double epsilon)
   {
      lowestMarginIndices.reset();

      if (!hasNearestConstraintEdge())
         return 0;

      int numberOfLowMarginEdges = 0;

      for (int edgeIndex = 0; edgeIndex < DIRECTIONS_TO_OPTIMIZE; edgeIndex++)
      {
         if (!Double.isNaN(comEdgeMargin[edgeIndex].getValue()) && comEdgeMargin[edgeIndex].getValue() < centerOfMassStabilityMargin.getValue() + epsilon)
         {
            int indexA = getVertexAOfEdge(edgeIndex);
            if (!lowestMarginIndices.contains(indexA))
               lowestMarginIndices.add(indexA);

            int indexB = getVertexBOfEdge(edgeIndex);
            if (!lowestMarginIndices.contains(indexB))
               lowestMarginIndices.add(indexB);

            numberOfLowMarginEdges++;
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
         YoGraphicLine2DDefinition nearestSegmentGraphic = YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition(namePrefix + "nearestConstraintViz",
                                                                                                                          nearestConstraintVertexA,
                                                                                                                          nearestConstraintVertexB,
                                                                                                                          lowestMarginEdgeGraphicColor);
         nearestSegmentGraphic.setStrokeWidth(2.5);
         group.addChild(nearestSegmentGraphic);
      }

      if (CenterOfMassStabilityMarginOptimizationModule.DEBUG)
      {
         group.addChild(optimizationModule.getSCS2YoGraphics());
      }

      return group;
   }
}
