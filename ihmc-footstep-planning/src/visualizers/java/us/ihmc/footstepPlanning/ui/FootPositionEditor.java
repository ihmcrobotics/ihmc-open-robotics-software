package us.ihmc.footstepPlanning.ui;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class FootPositionEditor extends AnimationTimer
{
   private static final boolean VERBOSE = true;

   private final EventHandler<MouseEvent> rayCastInterceptor;
   private boolean isRayCastInterceptorAttached = false;
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);

   private final AtomicBoolean positionValidated = new AtomicBoolean(false);

   private final Messager messager;
   private final Node sceneNode;

   private final AtomicReference<Boolean> editModeEnabled;
   private final MessagerAPIFactory.Topic<Point3D> positionTopic = FootstepPlannerUserInterfaceAPI.NodeCheckingPosition;

   public FootPositionEditor(Messager messager, Node sceneNode)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      this.editModeEnabled = messager.createInput(FootstepPlannerUserInterfaceAPI.EnableNodeChecking, false);

      rayCastInterceptor = new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            PickResult pickResult = event.getPickResult();
            Node intersectedNode = pickResult.getIntersectedNode();
            if (intersectedNode == null || intersectedNode instanceof SubScene)
               return;
            javafx.geometry.Point3D localPoint = pickResult.getIntersectedPoint();
            javafx.geometry.Point3D scenePoint = intersectedNode.getLocalToSceneTransform().transform(localPoint);

            Point3D interception = new Point3D();
            interception.setX(scenePoint.getX());
            interception.setY(scenePoint.getY());
            interception.setZ(scenePoint.getZ());

            latestInterception.set(interception);
         }
      };
   }

   @Override
   public void handle(long now)
   {
      if (editModeEnabled.get())
      {
         attachEvenHandlers();
      }
      else
      {
         removeEventHandlers();
         return;
      }

      if (editModeEnabled.get())
      {
         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            messager.submitMessage(positionTopic, interception);
            return;
         }
      }
   }

   private void attachEvenHandlers()
   {
      if (!isRayCastInterceptorAttached)
      {
         if (VERBOSE)
            PrintTools.info(this, "Attaching ray cast event handler.");
         sceneNode.addEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = true;
      }
   }

   private void removeEventHandlers()
   {
      if (isRayCastInterceptorAttached)
      {
         sceneNode.removeEventHandler(MouseEvent.ANY, rayCastInterceptor);
         isRayCastInterceptorAttached = false;
      }
   }

}
