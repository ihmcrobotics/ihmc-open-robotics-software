package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.awt.*;

public class PlanarRegionConstraintProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameConvexPolygon2d activePlanarRegion;

   private final PlanarRegionsList planarRegionsList = new PlanarRegionsList();

   public PlanarRegionConstraintProvider(String yoNamePrefix, boolean visualize, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      activePlanarRegion = new YoFrameConvexPolygon2d(yoNamePrefix + "ActivePlanarRegionConstraint", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon reachabilityGraphic = new YoArtifactPolygon("ActivePlanarRegionViz", activePlanarRegion, Color.RED, false);

         reachabilityGraphic.setVisible(visualize);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), reachabilityGraphic);
      }
   }

   public void setActivePlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionsList.clear();

      if (planarRegion != null)
      {
         planarRegionsList.addPlanarRegion(planarRegion);
         activePlanarRegion.setConvexPolygon2d(planarRegion.getConvexHull());
      }
      else
      {
         activePlanarRegion.clearAndHide();
      }
   }

   public void updatePlanarRegionConstraintForDoubleSupport(SimpleICPOptimizationQPSolver solver)
   {
      solver.resetPlanarRegionConstraint();
   }

   public void updatePlanarRegionConstraintForSingleSupport(SimpleICPOptimizationQPSolver solver)
   {
      solver.resetPlanarRegionConstraint();
      if (!planarRegionsList.isEmpty())
      {
         solver.setPlanarRegionConstraint(planarRegionsList.getLastPlanarRegion());
         activePlanarRegion.setConvexPolygon2d(planarRegionsList.getLastPlanarRegion().getConvexHull());
      }
   }

   public void reset()
   {
      activePlanarRegion.clearAndHide();
   }
}
