package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.GDXGraphics3DNode;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.tools.thread.Activator;

public class GDXRobotModelGraphic extends ImGuiGDXVisualizer implements RenderableProvider
{
   private GDXGraphics3DNode robotRootNode;
   private GraphicsRobot graphicsRobot;
   private Activator robotLoadedActivator = new Activator();

   public GDXRobotModelGraphic(String title)
   {
      super(title);
   }

   public void loadRobotModelAndGraphics(RobotDescription robotDescription, RigidBodyBasics rootBody)
   {
      if (robotRootNode != null)
         robotRootNode.destroy();

      graphicsRobot = new GraphicsIDRobot(robotDescription.getName(), rootBody, robotDescription);
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
   public void update()
   {
      super.update();
      if (robotLoadedActivator.poll())
      {
         graphicsRobot.update();
         robotRootNode.update();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
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

   public boolean isRobotLoaded()
   {
      return robotLoadedActivator.poll();
   }

   public GDXGraphics3DNode getRobotRootNode()
   {
      return robotRootNode;
   }

   public GraphicsRobot getGraphicsRobot()
   {
      return graphicsRobot;
   }
}
