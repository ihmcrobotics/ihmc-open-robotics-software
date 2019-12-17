package us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers;

import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Calculates Planar Region that corresponds to mouse position
 */
public class PlanarRegionSelector implements EventHandler<MouseEvent>
{
   private static final double selectorEpsilon = 1e-5;

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>();
   private final AtomicReference<PlanarRegion> selectedRegion = new AtomicReference<>();

   @Override
   public void handle(MouseEvent event)
   {
      PickResult pickResult = event.getPickResult();
      Node intersectedNode = pickResult.getIntersectedNode();
      if (!(intersectedNode instanceof MeshView))
         return;
      javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
      javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

      Point3D interception = new Point3D();
      interception.setX(scenePoint.getX());
      interception.setY(scenePoint.getY());
      interception.setZ(scenePoint.getZ());

      latestInterception.set(interception);

      if (planarRegionsList.get() != null)
      {
         PlanarRegion region = findRegion(planarRegionsList.get(), interception);
         if (region == null)
            return;

         selectedRegion.set(region);
      }
   }

   private static PlanarRegion findRegion(PlanarRegionsList planarRegionsList, Point3D point)
   {
      for (PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
      {
         if (PlanarRegionTools.isPointOnRegion(region, point, selectorEpsilon))
         {
            return region;
         }
      }
      return null;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.set(planarRegionsList);
   }

   public PlanarRegion getSelectedRegion()
   {
      return selectedRegion.get();
   }

   public Point3D pollSelectedPoint()
   {
      return latestInterception.getAndSet(null);
   }

}
