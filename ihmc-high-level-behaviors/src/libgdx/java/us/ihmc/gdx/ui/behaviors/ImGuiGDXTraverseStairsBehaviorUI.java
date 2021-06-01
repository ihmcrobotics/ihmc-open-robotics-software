package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;

public class ImGuiGDXTraverseStairsBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(TraverseStairsBehavior.DEFINITION,
                                                                                        ImGuiGDXTraverseStairsBehaviorUI::new);

   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private Point2D nodePosition = new Point2D(641.0, 369.0);

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
   public Point2D getTreeNodeInitialPosition()
   {
      return nodePosition;
   }

   @Override
   public void renderTreeNode()
   {

   }

   @Override
   public void renderInternal()
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

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
