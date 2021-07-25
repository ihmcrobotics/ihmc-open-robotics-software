package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXInteractableFoot
{
   private final GDXRobotCollisionLink collisionLink;
   private final RobotSide side;
   private final ReferenceFrame syncedRobotFootFrame;
   private final Model footModel;
   private final ModelInstance footModelInstance;
   private boolean selected = false;
   private boolean hovered;

   public GDXInteractableFoot(GDXRobotCollisionLink collisionLink, RobotSide side, ReferenceFrame syncedRobotFootFrame)
   {
      this.collisionLink = collisionLink;
      this.side = side;
      this.syncedRobotFootFrame = syncedRobotFootFrame;

      String robotSidePrefix = (side == RobotSide.LEFT) ? "l_" : "r_";
      String modelFileName = robotSidePrefix + "foot.g3dj";
      footModel = GDXModelLoader.loadG3DModel(modelFileName);
      footModelInstance = new ModelInstance(footModel);
      footModelInstance.transform.scale(1.01f, 1.01f, 1.01f);

      GDXTools.setTransparency(footModelInstance, 0.5f);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      hovered = !selected && collisionLink.getIntersects();
      if (hovered)
      {
         GDXTools.toGDX(syncedRobotFootFrame.getTransformToWorldFrame(), footModelInstance.transform);

         if (input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
         {
            selected = true;
         }
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (selected || hovered)
      {
         footModelInstance.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      footModel.dispose();
   }

   public GDXRobotCollisionLink getCollisionLink()
   {
      return collisionLink;
   }
}
