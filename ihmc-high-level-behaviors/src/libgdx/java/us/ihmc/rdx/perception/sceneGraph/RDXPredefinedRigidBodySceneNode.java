package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
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
public class RDXPredefinedRigidBodySceneNode
{
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
   private final PredefinedRigidBodySceneNode predefinedRigidBodySceneNode;
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private boolean showing = false;
   private final RDXModelInstance modelInstance;
   private final FramePose3D nodePose = new FramePose3D();
   private final RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();

   public RDXPredefinedRigidBodySceneNode(PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
   {
      this.predefinedRigidBodySceneNode = predefinedRigidBodySceneNode;

      modelInstance = new RDXModelInstance(RDXModelLoader.load(predefinedRigidBodySceneNode.getVisualModelFilePath()));
      modelInstance.setColor(GHOST_COLOR);

      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
   }

   public void update()
   {
      showing = predefinedRigidBodySceneNode.getCurrentlyDetected();

      referenceFrameGraphic.setToReferenceFrame(predefinedRigidBodySceneNode.getNodeFrame());

      nodePose.setToZero(predefinedRigidBodySceneNode.getNodeFrame());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());
      nodePose.get(nodeToWorldTransform);
      modelInstance.setTransformToWorldFrame(nodeToWorldTransform);
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
      return predefinedRigidBodySceneNode.getNodeToParentFrameTransform();
   }

   public void setTransformToParent(RigidBodyTransform transform)
   {
      predefinedRigidBodySceneNode.getNodeToParentFrameTransform().set(transform);
      predefinedRigidBodySceneNode.getNodeFrame().update();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return predefinedRigidBodySceneNode.getNodeFrame();
   }
}
