package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;

import java.util.ArrayList;
import java.util.List;

public class GDXRobotWholeBodyInteractable implements RenderableProvider
{
   private final RobotCollisionModel robotCollisionModel;
   private final FullHumanoidRobotModel fullRobotModel;

   private final ArrayList<GDXRobotCollisionLink> collisionLinks = new ArrayList<>();

   public GDXRobotWholeBodyInteractable(RobotCollisionModel robotCollisionModel, FullHumanoidRobotModel fullRobotModel)
   {
      this.robotCollisionModel = robotCollisionModel;
      this.fullRobotModel = fullRobotModel;
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      List<Collidable> robotCollidables = robotCollisionModel.getRobotCollidables(fullRobotModel.getElevator());
      for (Collidable collidable : robotCollidables)
      {
         GDXRobotCollisionLink collisionLink = new GDXRobotCollisionLink(collidable);
         collisionLinks.add(collisionLink);
         baseUI.get3DSceneManager().addRenderableProvider(collisionLink);
      }
   }

   public void update()
   {
      for (GDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.getRenderables(renderables, pool);
      }
   }
}
