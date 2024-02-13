package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.screwTheory.MassMatrixCalculator;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.awt.*;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginOptimizationModule.MAX_CONTACT_POINTS;

/**
 * Helper class for using {@link CenterOfMassStabilityMarginOptimizationModule} to update and manage a multi-contact stability region.
 */
public class CenterOfMassStaticStabilityRegionCalculator implements SCS2YoGraphicHolder
{
   private static final int DEFAULT_DIRECTIONS_TO_OPTIMIZE = 18;
   private static final boolean DEBUG = false;

   // TODO optimize how this is computed. Could use Bretls iterative projection algorithm,
   // For the case of roughly level feet + one hand, there might be a way to rule out query directions.
   // This is kind of an ad-hoc distribution to get a good initial query set
   private static final int[] vertexOrder = new int[]{0, 6, 12, 3, 9, 15, 1, 4, 7, 10, 13, 16, 2, 5, 8, 11, 14, 17};

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final int directionsToOptimize;
   private final double deltaAngle;
   private final YoInteger queryCounter = new YoInteger("queryIndex", registry);
   private final YoBoolean hasSolvedWholeRegion = new YoBoolean("hasSolvedWholeRegion", registry);
   private final YoFramePoint2D[] optimizedVertices;
   private final YoFrameConvexPolygon2D feasibleCoMRegion;
   private final CenterOfMassStabilityMarginOptimizationModule optimizationModule;
   private final YoFrameVector3D[] resolvedForces = new YoFrameVector3D[MAX_CONTACT_POINTS];

   public CenterOfMassStaticStabilityRegionCalculator(double robotMass)
   {
      this(robotMass, null, null);
   }

   public CenterOfMassStaticStabilityRegionCalculator(double robotMass, int directionsToOptimize)
   {
      this(robotMass, directionsToOptimize, null, null);
   }

   public CenterOfMassStaticStabilityRegionCalculator(double robotMass, YoRegistry parentRegistry)
   {
      this(robotMass, DEFAULT_DIRECTIONS_TO_OPTIMIZE, parentRegistry, null);
   }

   public CenterOfMassStaticStabilityRegionCalculator(double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(robotMass, DEFAULT_DIRECTIONS_TO_OPTIMIZE, parentRegistry, graphicsListRegistry);
   }

   public CenterOfMassStaticStabilityRegionCalculator(double robotMass, int directionsToOptimize, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      optimizationModule = new CenterOfMassStabilityMarginOptimizationModule(robotMass, registry, graphicsListRegistry);
      optimizedVertices = new YoFramePoint2D[directionsToOptimize];
      this.directionsToOptimize = directionsToOptimize;
      deltaAngle = 2.0 * Math.PI / directionsToOptimize;

      for (int i = 0; i < optimizedVertices.length; i++)
      {
         optimizedVertices[i] = new YoFramePoint2D("comStabilityMarginVertex" + i, ReferenceFrame.getWorldFrame(), registry);
      }
      for (int i = 0; i < resolvedForces.length; i++)
      {
         resolvedForces[i] = new YoFrameVector3D("resolvedForce" + i, ReferenceFrame.getWorldFrame(), registry);
      }

      if (DEBUG && graphicsListRegistry != null)
      {
         for (int contactIdx = 0; contactIdx < resolvedForces.length; contactIdx++)
         {
            graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(),
                                                   new YoGraphicVector("resolvedForceGraphic" + contactIdx, optimizationModule.getYoContactPointPosition(contactIdx), resolvedForces[contactIdx], 0.04 / robotMass, YoAppearance.Red()));
         }
      }

      feasibleCoMRegion = new YoFrameConvexPolygon2D("multiContactCoMRegion", ReferenceFrame.getWorldFrame(), directionsToOptimize, registry);
      feasibleCoMRegion.setToNaN();

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

      for (int vertexIdx = 0; vertexIdx < optimizedVertices.length; vertexIdx++)
      {
         optimizedVertices[vertexIdx].setToNaN();
      }
      for (int contactIdx = 0; contactIdx < resolvedForces.length; contactIdx++)
      {
         resolvedForces[contactIdx].setToNaN();
      }

      hasSolvedWholeRegion.set(false);
      queryCounter.set(0);
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
      int queryIndex = vertexOrder[queryCounter.getValue()];

      double queryX = Math.cos(queryIndex * deltaAngle);
      double queryY = Math.sin(queryIndex * deltaAngle);
      boolean success = optimizationModule.solve(queryX, queryY);

      if (!success)
      {
         optimizedVertices[queryIndex].setToNaN();
         hasSolvedWholeRegion.set(false);
         return false;
      }

      optimizedVertices[queryIndex].set(optimizationModule.getOptimizedCoM());
      hasSolvedWholeRegion.set(hasSolvedWholeRegion.getValue() || (queryCounter.getValue() == directionsToOptimize - 1));

      feasibleCoMRegion.clear();
      for (int queryIdx = 0; queryIdx < directionsToOptimize; queryIdx++)
      {
         if (!optimizedVertices[queryIdx].containsNaN())
            feasibleCoMRegion.addVertex(optimizedVertices[queryIdx]);
      }
      feasibleCoMRegion.update();

      for (int contactIdx = 0; contactIdx < optimizationModule.getNumberOfContactPoints(); contactIdx++)
      {
         optimizationModule.getResolvedForce(contactIdx, resolvedForces[contactIdx]);
      }

      queryCounter.set((queryCounter.getValue() + 1) % directionsToOptimize);

      return hasSolvedWholeRegion.getValue();
   }

   public int getNumberOfVertices()
   {
      return directionsToOptimize;
   }

   public boolean hasSolvedWholeRegion()
   {
      return hasSolvedWholeRegion.getValue();
   }

   public FrameConvexPolygon2DReadOnly getFeasibleCoMRegion()
   {
      return feasibleCoMRegion;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Multi-Contact CoM Region", feasibleCoMRegion, ColorDefinitions.Black(), false));
      return group;
   }
}
