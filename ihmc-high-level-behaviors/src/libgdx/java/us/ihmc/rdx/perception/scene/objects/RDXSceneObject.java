package us.ihmc.rdx.perception.scene.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.DetectableSceneObjectMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.scene.SceneObject;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.Set;

/**
 * A "ghost" colored model, with reference frame graphic in the virtual scene
 * to be used for debugging and know where the frame is.
 *
 * TODO: Add pose "override", via right click context menu, and gizmo.
 *   Possibly do this in a higher level class or class that extends this.
 */
public class RDXSceneObject extends SceneObject
{
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final ReferenceFrame referenceFrame;
   private boolean showing = false;
   private final RDXModelInstance modelInstance;
   private IHMCROS2Input<DetectableSceneObjectMessage> subscription;

   public RDXSceneObject(String modelName, String frameName)
   {
      super(frameName);

      modelInstance = new RDXModelInstance(RDXModelLoader.load(modelName));
      modelInstance.setColor(GHOST_COLOR);

      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(frameName, ReferenceFrame.getWorldFrame(), transformToParent);
      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
   }

   /**
    * Keeps this object updated from ROS 2 DetectableSceneObjectMessages.
    */
   public void setupForROS2Updating(ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectableSceneObjectMessage> updateTopic)
   {
      subscription = ros2.subscribe(updateTopic);
   }

   public void update()
   {
      if (subscription.getMessageNotification().poll())
      {
         DetectableSceneObjectMessage detectableSceneObjectMessage = subscription.getMessageNotification().read();
         if (getName().equals(detectableSceneObjectMessage.getNameAsString()))
         {
            setShowing(detectableSceneObjectMessage.getDetected());
            MessageTools.toEuclid(detectableSceneObjectMessage.getTransformToWorld(), getTransformToParent());
         }
      }

      referenceFrame.update();
      referenceFrameGraphic.setToReferenceFrame(referenceFrame);
      modelInstance.setTransformToWorldFrame(transformToParent);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showing)
      {
         if (sceneLevels.contains(RDXSceneLevel.MODEL))
            modelInstance.getRenderables(renderables, pool);

         if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
            referenceFrameGraphic.getRenderables(renderables, pool);
      }
   }

   public void setShowing(boolean showing)
   {
      this.showing = showing;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public void setTransformToParent(RigidBodyTransform transform)
   {
      transformToParent.set(transform);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
