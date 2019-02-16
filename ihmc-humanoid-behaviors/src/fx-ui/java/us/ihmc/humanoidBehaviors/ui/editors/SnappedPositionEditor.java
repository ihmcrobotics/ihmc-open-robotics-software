package us.ihmc.humanoidBehaviors.ui.editors;

import javafx.animation.AnimationTimer;
import javafx.scene.Node;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.ui.ChangingReference;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor.API.*;

public class SnappedPositionEditor extends FXUIEditor
{
   private final Messager messager;
   private final Node sceneNode;

   private final Topic<FXUIEditor> activeEditorTopic;

   private final ChangingReference<FXUIEditor> activeEditor;
   private final AtomicReference<PlanarRegionsList> planarRegionsList;
   private final AtomicBoolean userLeftClicked = new AtomicBoolean(false);
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);
   private final AtomicReference<PlanarRegion> selectedRegion = new AtomicReference<>(null);

//   private final Topic<Boolean> orientationEditModeEnabledTopic;

   public SnappedPositionEditor(Messager messager, Node sceneNode,
                                Topic<PlanarRegionsList> planarRegionDataTopic,
                                Topic<FXUIEditor> activeEditorTopic)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      this.activeEditorTopic = activeEditorTopic;

      activeEditor = new ChangingReference<>(messager.createInput(activeEditorTopic, FXUIEditor.NONE));
      planarRegionsList = messager.createInput(planarRegionDataTopic);
   }

   @Override
   public void handle(long now)
   {
      if (activeEditor.get() == this)
      {
         if (activeEditor.hasChanged())
         {
            sceneNode.addEventHandler(MouseEvent.MOUSE_MOVED, this::rayCastInterceptor);
            sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::leftClickInterceptor);
         }

         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            messager.submitMessage(SelectedPlanarRegion, selectedRegion.get());
            messager.submitMessage(SelectedPosition, interception);
         }

         if (userLeftClicked.getAndSet(false))
         {
            LogTools.debug("Start position is validated: " + interception, this);
            messager.submitMessage(activeEditorTopic, FXUIEditor.NONE);
//            if (orientationEditModeEnabledTopic != null)
//            {
//               LogTools.debug("submitMessage  startOrientationEditModeEnabledTopic");
//               messager.submitMessage(orientationEditModeEnabledTopic, true);
//            }
         }
      }
      else if (activeEditor.hasChanged())
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, this::rayCastInterceptor);
         sceneNode.removeEventHandler(MouseEvent.ANY, this::leftClickInterceptor);
      }
   }

   private void rayCastInterceptor(MouseEvent event)
   {
      PickResult pickResult = event.getPickResult();
      Node intersectedNode = pickResult.getIntersectedNode();
      if (intersectedNode == null || !(intersectedNode instanceof MeshView))
         return;
      javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
      javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

      Point3D interception = new Point3D();
      interception.setX(scenePoint.getX());
      interception.setY(scenePoint.getY());
      interception.setZ(scenePoint.getZ());

      latestInterception.set(interception);

      if (planarRegionsList != null)
      {
         PlanarRegion region = findRegion(planarRegionsList.get(), interception);
         if (region == null)
            return;

         selectedRegion.set(region);
      }
   }

   private void leftClickInterceptor(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY && event.isStillSincePress())
      {
         userLeftClicked.set(true);
      }
   }

   private static PlanarRegion findRegion(PlanarRegionsList planarRegionsList, Point3D point)
   {
      for (PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
      {
         if (PlanarRegionTools.isPointOnRegion(region, point, 1.0e-5))
         {
            return region;
         }
      }
      return null;
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Category = apiFactory.createRootCategory(apiFactory.createCategoryTheme(SnappedPositionEditor.class.getSimpleName()));
      private static final TopicTheme Theme = apiFactory.createTopicTheme("Default");

      public static final Topic<Point3D> SelectedPosition = Category.topic(Theme);
      public static final Topic<PlanarRegion> SelectedPlanarRegion = Category.topic(Theme);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
