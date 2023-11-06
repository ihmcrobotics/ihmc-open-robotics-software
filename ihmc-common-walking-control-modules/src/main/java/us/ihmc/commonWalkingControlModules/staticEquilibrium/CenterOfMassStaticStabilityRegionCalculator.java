package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class CenterOfMassStaticStabilityRegionCalculator
{
   private static final int DIRECTION_TO_OPTIMIZE = 20;
   private static final double DELTA_ANGLE = 2.0 * Math.PI / DIRECTION_TO_OPTIMIZE;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoInteger queryIndex = new YoInteger("queryIndex", registry);
   private final YoBoolean hasSolvedWholeRegion = new YoBoolean("hasSolvedWholeRegion", registry);
   private final YoFramePoint2D[] optimizedVertices = new YoFramePoint2D[DIRECTION_TO_OPTIMIZE];
   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();
   private final CenterOfMassStaticStabilityOptimizationModule optimizationModule;

   public CenterOfMassStaticStabilityRegionCalculator(double robotMass, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      optimizationModule = new CenterOfMassStaticStabilityOptimizationModule(robotMass, registry, graphicsListRegistry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void clear()
   {
      supportRegion.clear();

      for (int i = 0; i < optimizedVertices.length; i++)
      {
         optimizedVertices[i].setToNaN();
      }

      hasSolvedWholeRegion.set(false);
      queryIndex.set(0);
   }

   public void updateContactState(WholeBodyContactStateInterface contactState)
   {
      optimizationModule.updateContactState(contactState);
   }

   public boolean update()
   {
      double queryX = Math.cos(queryIndex.getValue() * DELTA_ANGLE);
      double queryY = Math.sin(queryIndex.getValue() * DELTA_ANGLE);
      boolean success = optimizationModule.solve(queryX, queryY);

      if (!success)
      {
         optimizedVertices[queryIndex.getValue()].setToNaN();
         hasSolvedWholeRegion.set(false);
         return false;
      }

      optimizedVertices[queryIndex.getValue()].set(optimizationModule.getOptimizedCoM());
      hasSolvedWholeRegion.set(hasSolvedWholeRegion.getValue() || (queryIndex.getValue() == DIRECTION_TO_OPTIMIZE - 1));

      supportRegion.clear();
      for (int queryIdx = 0; queryIdx < DIRECTION_TO_OPTIMIZE; queryIdx++)
      {
         if (!optimizedVertices[queryIdx].containsNaN())
            supportRegion.addVertex(optimizedVertices[queryIdx]);
      }
      supportRegion.update();

      queryIndex.set((queryIndex.getValue() + 1) % DIRECTION_TO_OPTIMIZE);
      return hasSolvedWholeRegion.getValue();
   }

   public boolean hasSolvedWholeRegion()
   {
      return hasSolvedWholeRegion.getValue();
   }

   public ConvexPolygon2DReadOnly getSupportRegion()
   {
      return supportRegion;
   }
}
