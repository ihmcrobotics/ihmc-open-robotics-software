package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class FrameGDXNode
{
   private final ReferenceFrame referenceFrame;
   private final DynamicGDXModel visualModel;
   private final DynamicGDXModel collisionModel;
   private final ModelInstance coordinateFrame;
   private final ReferenceFrame visualModelFrame;
   private final ModelInstance visualModelInstance;
   private ModelInstance collisionModelInstance;
   private ReferenceFrame collisionModelFrame;

   public FrameGDXNode(ReferenceFrame referenceFrame, DynamicGDXModel visualModel, DynamicGDXModel collisionModel)
   {
      this.referenceFrame = referenceFrame;
      this.visualModel = visualModel;
      this.collisionModel = collisionModel;

      visualModelFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent("visualModelFrame",
                                                                                                referenceFrame,
                                                                                                visualModel.getLocalTransform()); // TODO: Broken ?
      if (collisionModel != null)
      {
         collisionModelFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent("collisionModelFrame",
                                                                                                      referenceFrame,
                                                                                                      collisionModel.getLocalTransform());
         collisionModelInstance = collisionModel.getOrCreateModelInstance();
      }

      visualModelInstance = visualModel.getOrCreateModelInstance();

      coordinateFrame = GDXModelPrimitives.createCoordinateFrameInstance(0.15);
   }

   public void updatePose()
   {
      visualModelFrame.update();
//      GDXTools.toGDX(visualModelFrame.getTransformToRoot(), visualModel.getOrCreateModelInstance().transform);
//      GDXTools.toGDX(referenceFrame.getTransformToRoot(), visualModel.getOrCreateModelInstance().transform);
      GDXTools.toGDX(referenceFrame.getTransformToRoot(), visualModelInstance.transform);

      if (collisionModel != null)
      {
         collisionModelFrame.update();
         GDXTools.toGDX(collisionModelFrame.getTransformToRoot(), collisionModelInstance.transform);
      }

      GDXTools.toGDX(referenceFrame.getTransformToRoot(), coordinateFrame.transform);
   }

   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      visualModelInstance.getRenderables(renderables, pool);
      if (collisionModel != null)
         collisionModelInstance.getRenderables(renderables, pool);

      coordinateFrame.getRenderables(renderables, pool);
   }
}
