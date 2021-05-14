package us.ihmc.quadrupedFootstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import javafx.event.EventHandler;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class NodeCheckerEditor extends AnimationTimer
{
   private static final boolean VERBOSE = true;

   private final EventHandler<MouseEvent> rayCastInterceptor;
   private boolean isRayCastInterceptorAttached = false;
   private final AtomicReference<Point3D> latestInterception = new AtomicReference<>(null);

   private final Messager messager;
   private final Node sceneNode;

   private final AtomicReference<Boolean> moduleEnabled;
   private final AtomicReference<Boolean> positionEditingEnabled;
   private final MessagerAPIFactory.Topic<Point3D> positionTopic = PawStepPlannerMessagerAPI.NodeCheckingPosition;

   private final AtomicBoolean positionValidated = new AtomicBoolean(false);

   public NodeCheckerEditor(Messager messager, Node sceneNode)
   {
      this.messager = messager;
      this.sceneNode = sceneNode;

      this.moduleEnabled = messager.createInput(PawStepPlannerMessagerAPI.EnableNodeChecking, false);
      this.positionEditingEnabled = messager.createInput(PawStepPlannerMessagerAPI.EnableNodeCheckingPositionEditing, false);

      rayCastInterceptor = event ->
      {
         if (event.isStillSincePress() && event.getEventType() == MouseEvent.MOUSE_CLICKED)
            positionValidated.set(true);

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
      };
   }

   @Override
   public void handle(long now)
   {
      if (moduleEnabled.get())
      {
         Point3D interception = latestInterception.getAndSet(null);
         if (interception != null)
         {
            messager.submitMessage(positionTopic, interception);
         }
      }

      if (positionEditingEnabled.get())
      {
         attachEvenHandlers();

         if(positionValidated.getAndSet(false))
            positionEditingEnabled.set(false);
      }
      else
      {
         removeEventHandlers();
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
