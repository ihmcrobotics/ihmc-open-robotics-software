package us.ihmc.rdx.ui.collidables;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.affordances.RDXRobotCollidable;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;

import java.util.ArrayList;
import java.util.List;

public class RDXRobotCollisionModel
{
   private final RobotCollisionModel robotCollisionModel;
   private final ArrayList<RDXRobotCollidable> robotCollidables = new ArrayList<>();
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
      for (int i = 0; i < collidables.size(); i++)
      {
         RDXRobotCollidable robotCollidable = new RDXRobotCollidable(collidables.get(i), i, LibGDXTools.toLibGDX(color));
         robotCollidables.add(robotCollidable);
      }
   }

   public void update()
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.update();
      }
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.calculateVRPick(vrContext);
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.processVRInput(vrContext);
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.calculatePick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.process3DViewInput(input);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXRobotCollidable robotCollidable : robotCollidables)
      {
         robotCollidable.getRenderables(renderables, pool);
      }
   }

   public ArrayList<RDXRobotCollidable> getRobotCollidables()
   {
      return robotCollidables;
   }
}
