package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class QuadrupedAStarFootstepPlannerVisualizer implements QuadrupedFootstepPlannerListener
{
   private static final int maxNumberOfChildNodes = 500;
   private static final double childNodeSize = 0.005;
   private static final double nodeSize = 0.007;

   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
   private final YoDouble time = new YoDouble("time", registry);

   private final HashMap<FootstepNode, List<FootstepNode>> validChildNodeMap = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> invalidChildNodeMap = new HashMap<>();

   private final List<YoFramePoint2D> yoValidChildNodes = new ArrayList<>();
   private final List<YoFramePoint2D> yoInValidChildNodes = new ArrayList<>();

   private final YoFootstepNode currentFootstepNode = new YoFootstepNode(registry);

   public QuadrupedAStarFootstepPlannerVisualizer(PlanarRegionsList regions)
   {
      this(0.1, regions);
   }

   public QuadrupedAStarFootstepPlannerVisualizer(double playbackRate, PlanarRegionsList regions)
   {
      for (int i = 0; i < maxNumberOfChildNodes; i++)
      {
         YoFramePoint2D validChildNode = new YoFramePoint2D("validChildNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition validChildNodeVis = new YoGraphicPosition("validChildNode" + i, validChildNode, childNodeSize, YoAppearance.Green());

         yoValidChildNodes.add(validChildNode);
         graphicsListRegistry.registerYoGraphic("plannerListener", validChildNodeVis);

         YoFramePoint2D invalidChildNode = new YoFramePoint2D("invalidChildNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition invalidChildNodeVis = new YoGraphicPosition("invalidChildNode" + i, invalidChildNode, childNodeSize, YoAppearance.Red());

         yoInValidChildNodes.add(invalidChildNode);
         graphicsListRegistry.registerYoGraphic("plannerListener", invalidChildNodeVis);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoGraphicPosition otherFeet = new YoGraphicPosition(robotQuadrant.getShortName() + "activeNodeFeet", currentFootstepNode.getYoPosition(robotQuadrant), nodeSize, YoAppearance.Orange());
         graphicsListRegistry.registerYoGraphic("plannerListener", otherFeet);

      }
      YoGraphicPosition nodeVis = new YoGraphicPosition("activeNode", currentFootstepNode.getMovingYoPosition(), nodeSize, YoAppearance.White());

      graphicsListRegistry.registerYoGraphic("plannerListener", nodeVis);

      YoGraphicReferenceFrame worldViz = new YoGraphicReferenceFrame(worldFrame, registry, false, 0.5);
      graphicsListRegistry.registerYoGraphic("plannerListener", worldViz);

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
      scs.setGroundVisible(false);
      scs.tickAndUpdate();
   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
         return;

      if (!validChildNodeMap.containsKey(previousNode))
         validChildNodeMap.put(previousNode, new ArrayList<>());

      validChildNodeMap.get(previousNode).add(node);

      currentFootstepNode.set(previousNode);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason reason)
   {
      if(validChildNodeMap.containsKey(parentNode))
         validChildNodeMap.get(parentNode).remove(rejectedNode);

      if (!invalidChildNodeMap.containsKey(parentNode))
         invalidChildNodeMap.put(parentNode, new ArrayList<>());

      invalidChildNodeMap.get(parentNode).add(rejectedNode);
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, QuadrupedFootstepPlannerNodeRejectionReason reason)
   {
      // TODO
   }

   @Override
   public void plannerFinished(List<FootstepNode> plan)
   {

   }

   @Override
   public void reportLowestCostNodeList(List<FootstepNode> plan)
   {

   }

   @Override
   public void tickAndUpdate()
   {
      yoValidChildNodes.forEach(YoFramePoint2D::setToNaN);

      FootstepNode currentNode = currentFootstepNode.getEquivalentNode();

      if (currentNode == null)
         return;


      List<FootstepNode> validChildNodes = validChildNodeMap.get(currentNode);
      if (validChildNodes != null)
      {
         for (int i = 0; i < validChildNodes.size(); i++)
         {
            FootstepNode childNode = validChildNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoValidChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }

      List<FootstepNode> invalidChildNodes = invalidChildNodeMap.get(currentNode);
      if (invalidChildNodes != null)
      {
         for (int i = 0; i < invalidChildNodes.size(); i++)
         {
            FootstepNode childNode = invalidChildNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoInValidChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }

      scs.setTime(time.getDoubleValue());
      scs.tickAndUpdate();
      time.add(1.0);
   }

   public void showAndSleep(boolean autoplay)
   {
      if (autoplay)
         scs.play();
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }
}
