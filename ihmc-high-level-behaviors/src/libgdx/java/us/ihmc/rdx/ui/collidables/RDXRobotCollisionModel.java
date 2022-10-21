package us.ihmc.rdx.ui.collidables;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.affordances.RDXRobotCollisionLink;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;

import java.util.ArrayList;
import java.util.List;

public class RDXRobotCollisionModel
{
   private final RobotCollisionModel robotCollisionModel;
   private final ArrayList<RDXRobotCollisionLink> collisionLinks = new ArrayList<>();
   private List<Collidable> collidables;

   public RDXRobotCollisionModel(RobotCollisionModel robotCollisionModel)
   {
      this.robotCollisionModel = robotCollisionModel;
   }

   public void create(ROS2SyncedRobotModel syncedRobot, AppearanceDefinition color)
   {
      create(syncedRobot.getFullRobotModel().getElevator(), color);
   }

   public void create(RigidBodyBasics rootBody, AppearanceDefinition color)
   {
      collidables = robotCollisionModel.getRobotCollidables(rootBody);
      for (Collidable collidable : collidables)
      {
         RDXRobotCollisionLink collisionLink = new RDXRobotCollisionLink(collidable, LibGDXTools.toGDX(color));
         collisionLinks.add(collisionLink);
      }
   }

   public void update()
   {
      for (RDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.update();
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.calculatePick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.process3DViewInput(input);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXRobotCollisionLink collisionLink : collisionLinks)
      {
         collisionLink.getRenderables(renderables, pool);
      }
   }

   public ArrayList<RDXRobotCollisionLink> getCollisionLinks()
   {
      return collisionLinks;
   }

   public List<Collidable> getCollidables()
   {
      return collidables;
   }
}
