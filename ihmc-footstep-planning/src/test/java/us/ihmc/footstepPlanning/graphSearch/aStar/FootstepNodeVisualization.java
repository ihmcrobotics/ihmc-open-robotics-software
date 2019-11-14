package us.ihmc.footstepPlanning.graphSearch.aStar;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.List;
import java.util.Queue;

public class FootstepNodeVisualization implements BipedalFootstepPlannerListener
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble time = new YoDouble("Time", registry);

   private final HashMap<FootstepNode, YoGraphicPosition> activeNodes = new HashMap<>();
   private final HashMap<FootstepNode, YoGraphicPosition> inactiveNodes = new HashMap<>();

   private final YoInteger nodeCount = new YoInteger("NodeCount", registry);

   private final Queue<YoGraphicPosition> activeNodeGraphicsQueue = new ArrayDeque<>();
   private final Queue<YoGraphicPosition> inactiveNodeGraphicsQueue = new ArrayDeque<>();

   public FootstepNodeVisualization()
   {
      this(1000, 0.01, null);
   }

   public FootstepNodeVisualization(int maxNodes, double playbackRate, PlanarRegionsList regions)
   {
      nodeCount.set(0);
      time.set(0.0);

      String listName = getClass().getSimpleName();
      for (int i = 0; i < maxNodes; i++)
      {
         YoFramePoint3D yoPoint = new YoFramePoint3D("ActiveNode" + i, worldFrame, registry);
         yoPoint.setToNaN();
         YoGraphicPosition yoGraphic = new YoGraphicPosition("ActiveNode " + i, yoPoint, 0.025, YoAppearance.Green());
         activeNodeGraphicsQueue.add(yoGraphic);
         graphicsListRegistry.registerYoGraphic("Active" + listName, yoGraphic);
      }

      for (int i = 0; i < maxNodes; i++)
      {
         YoFramePoint3D yoPoint = new YoFramePoint3D("InactiveNode" + i, worldFrame, registry);
         yoPoint.setToNaN();
         YoGraphicPosition yoGraphic = new YoGraphicPosition("InactiveNode " + i, yoPoint, 0.025, YoAppearance.Red());
         inactiveNodeGraphicsQueue.add(yoGraphic);
         graphicsListRegistry.registerYoGraphic("Inactive" + listName, yoGraphic);
      }

      YoGraphicReferenceFrame worldViz = new YoGraphicReferenceFrame(worldFrame, registry, false, 0.5);
      graphicsListRegistry.registerYoGraphic(listName, worldViz);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      if (regions != null)
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, regions);
      scs.addStaticLinkGraphics(graphics3DObject);

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(playbackRate);
      scs.setMaxBufferSize(64000);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.001, 0.0, 15.0);
      scs.tickAndUpdate();
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      FootstepNode localNode = creat2dNode(node);

      if (nodeExists(localNode))
      {
         setNodeActive(localNode);
         return;
      }

      addNodeUnsafe(localNode, true);
   }

   private void addNodeUnsafe(FootstepNode node, boolean active)
   {
      if (active)
      {
         YoGraphicPosition graphics = activeNodeGraphicsQueue.poll();
         setGraphics(graphics, node);
         activeNodes.put(node, graphics);
      }
      else
      {
         YoGraphicPosition graphics = inactiveNodeGraphicsQueue.poll();
         setGraphics(graphics, node);
         inactiveNodes.put(node, graphics);
      }

      nodeCount.increment();
   }

   private void setNodeActive(FootstepNode node)
   {
      FootstepNode localNode = creat2dNode(node);

      if (nodeExists(localNode))
      {
         if (isNodeActive(localNode))
            return;

         YoGraphicPosition inactiveDisplay = inactiveNodes.remove(localNode);
         YoGraphicPosition activeDisplay = activeNodeGraphicsQueue.poll();

         hideGraphics(inactiveDisplay);
         setGraphics(activeDisplay, localNode);

         activeNodes.put(localNode, activeDisplay);
         inactiveNodeGraphicsQueue.add(inactiveDisplay);
      }
      else
         addNodeUnsafe(localNode, true);
   }

   @Override
   public void rejectNode(FootstepNode node, FootstepNode previousNode, BipedalFootstepPlannerNodeRejectionReason reason)
   {
      FootstepNode localNode = creat2dNode(node);

      if (nodeExists(localNode))
      {
         if (!isNodeActive(localNode))
            return;

         YoGraphicPosition inactiveDisplay = activeNodes.remove(localNode);
         YoGraphicPosition activeDisplay = inactiveNodeGraphicsQueue.poll();

         hideGraphics(inactiveDisplay);
         setGraphics(activeDisplay, localNode);

         inactiveNodes.put(localNode, activeDisplay);
         activeNodeGraphicsQueue.add(inactiveDisplay);
      }
      else
         addNodeUnsafe(localNode, false);
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {
   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {
   }

   private boolean nodeExists(FootstepNode node)
   {
      FootstepNode localNode = creat2dNode(node);

      if (activeNodes.containsKey(localNode))
         return true;
      if (inactiveNodes.containsKey(localNode))
         return true;
      return false;
   }

   private boolean isNodeActive(FootstepNode node)
   {
      if (activeNodes.containsKey(node))
         return true;
      if (inactiveNodes.containsKey(node))
         return false;
      throw new RuntimeException("Node does not exist.");
   }

   private void setGraphics(YoGraphicPosition graphics, FootstepNode node)
   {
      graphics.setPosition(node.getX(), node.getY(), 0.0);
      graphics.update();
   }

   private void hideGraphics(YoGraphicPosition graphics)
   {
      graphics.setPositionToNaN();
      graphics.update();
   }

   public void showAndSleep(boolean autoplay)
   {
      if (autoplay)
         scs.play();
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   @Override
   public void tickAndUpdate()
   {
      scs.setTime(time.getDoubleValue());
      scs.tickAndUpdate();
      time.add(1.0);
   }

   private FootstepNode creat2dNode(FootstepNode node)
   {
      return new FootstepNode(node.getX(), node.getY());
   }

   /**
    * Test method that makes a pattern of active and inactive nodes just for demonstration.
    * @param args
    */
   public static void main(String[] args)
   {
      FootstepNodeVisualization viz = new FootstepNodeVisualization(100, 0.01, null);

      for (int i = 0; i < 10; i++)
      {
         viz.addNode(new FootstepNode(0.05 * i, 0.0), null);
         viz.tickAndUpdate();
      }

      for (int i = 0; i < 10; i++)
      {
         viz.addNode(new FootstepNode(0.05 * i, 0.1), null);
         viz.tickAndUpdate();
      }

      for (int i = 0; i < 10; i++)
      {
         FootstepNode node = new FootstepNode(0.05 * i, 0.0);
         viz.rejectNode(node, null, null);
         viz.tickAndUpdate();
      }

      for (int i = 0; i < 10; i++)
      {
         FootstepNode node = new FootstepNode(0.05 * i, 0.1);
         viz.rejectNode(node, null, null);
         viz.tickAndUpdate();
      }

      for (int i = 0; i < 10; i++)
      {
         viz.addNode(new FootstepNode(0.05 * i, 0.0), null);
         viz.tickAndUpdate();
      }

      for (int i = 0; i < 10; i++)
      {
         viz.addNode(new FootstepNode(0.05 * i, 0.1), null);
         viz.tickAndUpdate();
      }

      viz.showAndSleep(true);
   }
}
