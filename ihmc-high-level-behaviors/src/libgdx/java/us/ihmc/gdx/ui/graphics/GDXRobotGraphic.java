package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.gdx.GDXGraphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.tools.thread.Activator;

public class GDXRobotGraphic implements RenderableProvider
{
   protected final FullHumanoidRobotModel fullRobotModel;
   protected GraphicsRobot graphicsRobot;
   protected GDXGraphics3DNode robotRootNode;
   protected Activator robotLoadedActivator = new Activator();

   public GDXRobotGraphic(DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;

      // this doesn't work because you have to load GDX models in the same thread as the context or something
      //      scheduler.scheduleOnce(() -> loadRobotModelAndGraphics(robotModel.getRobotDescription()));
      loadRobotModelAndGraphics(robotModel.getRobotDescription());
   }

   public void update()
   {
      if (robotLoadedActivator.poll())
      {
         graphicsRobot.update();
         robotRootNode.update();
      }
   }

   private void loadRobotModelAndGraphics(RobotDescription robotDescription)
   {
      graphicsRobot = new GraphicsIDRobot(robotDescription.getName(), fullRobotModel.getElevator(), robotDescription);
      robotRootNode = new GDXGraphics3DNode(graphicsRobot.getRootNode());
      //      robotRootNode.setMouseTransparent(true);
      addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode);
      robotRootNode.update();

      robotLoadedActivator.activate();
   }

   private void addNodesRecursively(Graphics3DNode graphics3DNode, GDXGraphics3DNode parentNode)
   {
      GDXGraphics3DNode node = new GDXGraphics3DNode(graphics3DNode);
      parentNode.addChild(node);
      graphics3DNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (robotLoadedActivator.poll())
      {
         robotRootNode.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      robotRootNode.destroy();
   }
}
