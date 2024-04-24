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
import java.util.Arrays;
import java.util.function.Supplier;

/**
 * Helper class for using {@link CenterOfMassStabilityMarginOptimizationModule} to update and manage a multi-contact stability region.
 */
public class CenterOfMassStaticStabilityRegionCalculator implements SCS2YoGraphicHolder
{
   private static final boolean DEBUG = false;
   private static final int NULL_NEAREST_INDEX = -1;

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
   private final TIntArrayList[] optimizerActiveSets = new TIntArrayList[DIRECTIONS_TO_OPTIMIZE];
   private final YoFramePoint2D nearestConstraintVertex1;
   private final YoFramePoint2D nearestConstraintVertex2;
   private final YoInteger nearestConstraintQueryIndex;
   private final YoDouble centerOfMassStabilityMargin;

   private Supplier<FramePoint3DReadOnly> centerOfMassSupplier = null;
   private boolean showNearestSupportEdgeGraphic = true;

   public CenterOfMassStaticStabilityRegionCalculator(String namePrefix, double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
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
         optimizerActiveSets[query_idx] = new TIntArrayList();
      }

      feasibleCoMRegion = new YoFrameConvexPolygon2D("multiContactCoMRegion", ReferenceFrame.getWorldFrame(), DIRECTIONS_TO_OPTIMIZE, registry);
      feasibleCoMRegion.setToNaN();

      nearestConstraintVertex1 = new YoFramePoint2D("nearestConstraintVertex1", ReferenceFrame.getWorldFrame(), registry);
      nearestConstraintVertex2 = new YoFramePoint2D("nearestConstraintVertex2", ReferenceFrame.getWorldFrame(), registry);
      nearestConstraintQueryIndex = new YoInteger("nearestConstraintQueryIndex", registry);
      centerOfMassStabilityMargin = new YoDouble("centerOfMassStabilityMargin", registry);
      nearestConstraintQueryIndex.set(NULL_NEAREST_INDEX);
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
         optimizerActiveSets[query_idx].reset();
      }

      hasSolvedWholeRegion.set(false);
      queryCounter.set(0);

      nearestConstraintVertex1.setToNaN();
      nearestConstraintVertex2.setToNaN();
      nearestConstraintQueryIndex.set(NULL_NEAREST_INDEX);
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

   public void performCoMRegionQuery()
   {
      int queryIndex = VERTEX_ORDER[queryCounter.getValue()];
      if (performCoMRegionQuery(queryIndex))
         queryCounter.set((queryCounter.getValue() + 1) % DIRECTIONS_TO_OPTIMIZE);
   }

   public boolean performCoMRegionQuery(int queryIndex)
   {
      double queryX = Math.cos(queryIndex * deltaAngle);
      double queryY = Math.sin(queryIndex * deltaAngle);
      boolean success = optimizationModule.solve(queryX, queryY);

      if (success)
      {
         optimizerActiveSets[queryIndex].reset();
         optimizerActiveSets[queryIndex].addAll(optimizationModule.getLinearProgramSolver().getSimplexStatistics().getActiveSetIndices());

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
      }
      else
      {
         optimizedVertices[queryIndex].setToNaN();
         resolvedForces[queryIndex].zero();
         optimizerActiveSets[queryIndex].reset();
         hasSolvedWholeRegion.set(false);
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

      if (centerOfMassStabilityMargin != null)
      {
         updateNearestVertex(getPreviousIndex(queryIndex));
         updateNearestVertex(queryIndex);
      }

      return true;
   }

   private void updateNearestVertex(int queryIndex)
   {
      YoFramePoint2D v0 = optimizedVertices[queryIndex];
      YoFramePoint2D v1 = optimizedVertices[getNextIndex(queryIndex)];

      if (v0.containsNaN() || v1.containsNaN())
      {
         return;
      }

      FramePoint3DReadOnly centerOfMass = centerOfMassSupplier.get();
      centerOfMass.getReferenceFrame().checkReferenceFrameMatch(v0.getReferenceFrame());

      double margin = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(centerOfMass.getX(), centerOfMass.getY(), v0, v1);
      if (margin < centerOfMassStabilityMargin.getValue())
      {
         centerOfMassStabilityMargin.set(margin);
         nearestConstraintQueryIndex.set(queryIndex);
         nearestConstraintVertex1.set(v0);
         nearestConstraintVertex2.set(v1);
      }
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
      return nearestConstraintQueryIndex.getValue() != NULL_NEAREST_INDEX;
   }

   public TIntArrayList getLowestMarginActiveSet1()
   {
      return hasNearestConstraintEdge() ? optimizerActiveSets[nearestConstraintQueryIndex.getValue()] : null;
   }

   public TIntArrayList getLowestMarginActiveSet2()
   {
      return hasNearestConstraintEdge() ? optimizerActiveSets[getPreviousIndex(nearestConstraintQueryIndex.getValue())] : null;
   }

   public DMatrixRMaj getLowestMarginResolvedForce1()
   {
      return hasNearestConstraintEdge() ? resolvedForces[nearestConstraintQueryIndex.getValue()] : null;
   }

   public DMatrixRMaj getLowestMarginResolvedForce2()
   {
      return hasNearestConstraintEdge() ? resolvedForces[getPreviousIndex(nearestConstraintQueryIndex.getValue())] : null;
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
         YoGraphicLine2DDefinition nearestSegmentGraphic = YoGraphicDefinitionFactory.newYoGraphicLineSegment2DDefinition(namePrefix + "nearestConstraintViz", nearestConstraintVertex1, nearestConstraintVertex2, lowestMarginEdgeGraphicColor);
         nearestSegmentGraphic.setStrokeWidth(2.5);
         group.addChild(nearestSegmentGraphic);
      }
      return group;
   }
}
