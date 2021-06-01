package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;

public class ImGuiGDXTraverseStairsBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(TraverseStairsBehavior.DEFINITION,
                                                                                        ImGuiGDXTraverseStairsBehaviorUI::new);

   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private Point2D32 nodePosition = new Point2D32(50.0f, 50.0f);

   public ImGuiGDXTraverseStairsBehaviorUI(BehaviorHelper helper)
   {
      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.PLANNED_STEPS, footsteps ->
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps));
      });
      footstepPlanGraphic.setTransparency(0.5);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {

   }

   @Override
   public Point2D32 getNodePosition(String nodeName)
   {
      return nodePosition;
   }

   @Override
   public void renderNode(String nodeName)
   {
      if (nodeName.equals(DEFINITION.getName()))
      {
         render();
      }
   }

   @Override
   public void render()
   {
      footstepPlanGraphic.render();
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
   }
}
