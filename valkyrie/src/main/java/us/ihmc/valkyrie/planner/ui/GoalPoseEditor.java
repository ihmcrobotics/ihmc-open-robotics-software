package us.ihmc.valkyrie.planner.ui;

import javafx.animation.AnimationTimer;
import javafx.beans.property.ObjectProperty;
import javafx.event.EventHandler;
import javafx.scene.SubScene;
import javafx.scene.input.KeyCode;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers.PlanarRegionSelector;
import us.ihmc.pathPlanning.visibilityGraphs.ui.eventHandlers.PlaneIntersectionCalculator;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class GoalPoseEditor extends AnimationTimer
{
   private final SubScene subScene;
   private final PlanarRegionSelector planarRegionSelector = new PlanarRegionSelector();
   private final PlaneIntersectionCalculator planeIntersectionCalculator;

   private final AtomicBoolean leftButtonClicked = new AtomicBoolean();
   private final EventHandler<MouseEvent> leftClickHandler = mouseEvent ->
   {
      if (mouseEvent.getButton() != MouseButton.PRIMARY)
         return;

      if (mouseEvent.isStillSincePress() && mouseEvent.getEventType() == MouseEvent.MOUSE_CLICKED)
         leftButtonClicked.set(true);
   };

   private final AtomicBoolean enable = new AtomicBoolean();
   private final AtomicBoolean positionValidated = new AtomicBoolean();

   private final ObjectProperty<Double> xProperty;
   private final ObjectProperty<Double> yProperty;
   private final ObjectProperty<Double> zProperty;
   private final ObjectProperty<Double> yawProperty;

   public GoalPoseEditor(SubScene subScene,
                         ObjectProperty<Double> xProperty,
                         ObjectProperty<Double> yProperty,
                         ObjectProperty<Double> zProperty,
                         ObjectProperty<Double> yawProperty)
   {
      this.subScene = subScene;
      this.xProperty = xProperty;
      this.yProperty = yProperty;
      this.zProperty = zProperty;
      this.yawProperty = yawProperty;

      planeIntersectionCalculator = new PlaneIntersectionCalculator(subScene.getCamera());
      subScene.setOnKeyPressed(keyEvent ->
                               {
                                  if (keyEvent.getCode() == KeyCode.G)
                                     enable();
                               });
   }

   @Override
   public void handle(long now)
   {
      if (!enable.get())
      {
         return;
      }

      if(!positionValidated.get())
      {
         Point3D goalPosition = planarRegionSelector.pollSelectedPoint();
         if(goalPosition != null)
         {
            xProperty.setValue(goalPosition.getX());
            yProperty.setValue(goalPosition.getY());
            zProperty.setValue(goalPosition.getZ());
         }
      }
      else
      {
         Point3D goalOrientationPoint = planeIntersectionCalculator.pollIntersection();
         if(goalOrientationPoint != null)
         {
            Vector3D difference = new Vector3D();
            difference.sub(goalOrientationPoint, new Vector3D(xProperty.getValue(), yProperty.getValue(), zProperty.getValue()));
            double goalYaw = Math.atan2(difference.getY(), difference.getX());
            yawProperty.set(goalYaw);
         }
      }

      if (leftButtonClicked.getAndSet(false))
      {
         if (!positionValidated.getAndSet(true))
         {
            subScene.removeEventHandler(MouseEvent.ANY, planarRegionSelector);
            if (planarRegionSelector.getSelectedRegion() == null)
            {
               enable.set(false);
               return;
            }
            planeIntersectionCalculator.setPlanarRegion(planarRegionSelector.getSelectedRegion());
            subScene.addEventHandler(MouseEvent.ANY, planeIntersectionCalculator);
         }
         else
         {
            subScene.removeEventHandler(MouseEvent.ANY, leftClickHandler);
            subScene.removeEventHandler(MouseEvent.ANY, planeIntersectionCalculator);
            enable.set(false);
         }
      }
   }

   public void enable()
   {
      if (!enable.get())
      {
         subScene.addEventHandler(MouseEvent.ANY, planarRegionSelector);
         enable.set(true);
         leftButtonClicked.set(false);
         positionValidated.set(false);
         subScene.addEventHandler(MouseEvent.ANY, leftClickHandler);
      }
   }

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      planarRegionSelector.setPlanarRegionsList(planarRegions);
   }
}
