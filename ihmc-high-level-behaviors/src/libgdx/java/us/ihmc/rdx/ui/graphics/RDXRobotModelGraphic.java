package us.ihmc.rdx.ui.graphics;

import java.util.concurrent.atomic.AtomicReference;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.RDXGraphics3DNode;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.description.RobotDefinitionConverter;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.tools.thread.Activator;

@Deprecated // Use RDXMultiBodyGraphic instead.
public class RDXRobotModelGraphic extends RDXVisualizer implements RenderableProvider
{
   private RDXGraphics3DNode robotRootNode;
   private GraphicsRobot graphicsRobot;
   private final Activator robotLoadedActivator = new Activator();

   public RDXRobotModelGraphic(String title)
   {
      super(title);
   }

   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics rootBody)
   {
      loadRobotModelAndGraphics(robotDefinition, rootBody, new Object());
   }

   // syncObject is so that robots can be loaded in parallel but also with different default textures
   // This sucks and SDFGraphics3DObject.DEFAULT_APPEARANCE is a terrible design
   public void loadRobotModelAndGraphics(RobotDefinition robotDefinition, RigidBodyBasics rootBody, Object syncObject)
   {
      if (robotRootNode != null)
         robotRootNode.destroy();

      ThreadTools.startAsDaemon(() ->
      {
         synchronized (syncObject)
         {
            graphicsRobot = new GraphicsIDRobot(robotDefinition.getName(), rootBody, RobotDefinitionConverter.toGraphicsObjectsHolder(robotDefinition));
            robotRootNode = new RDXGraphics3DNode(graphicsRobot.getRootNode());
            //      robotRootNode.setMouseTransparent(true);
            addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode);
            robotRootNode.update();
            robotLoadedActivator.activate();
         }
      }, "RobotModelLoader");
   }

   private void addNodesRecursively(Graphics3DNode graphics3DNode, RDXGraphics3DNode parentNode)
   {
      AtomicReference<RDXGraphics3DNode> node = new AtomicReference<>();
      Gdx.app.postRunnable(() ->
      {
         node.set(new RDXGraphics3DNode(graphics3DNode));
      });

      while (node.get() == null)
      {
         ThreadTools.sleep(100);
      }

      parentNode.addChild(node.get());
      graphics3DNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node.get()));
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
      if (isActive() && robotLoadedActivator.poll())
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

   public RDXGraphics3DNode getRobotRootNode()
   {
      return robotRootNode;
   }

   public GraphicsRobot getGraphicsRobot()
   {
      return graphicsRobot;
   }
}
