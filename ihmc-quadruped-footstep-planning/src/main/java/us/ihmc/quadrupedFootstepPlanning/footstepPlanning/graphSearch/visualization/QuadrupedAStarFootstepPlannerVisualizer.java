package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.*;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class QuadrupedAStarFootstepPlannerVisualizer implements QuadrupedFootstepPlannerListener
{
   private static final int maxNumberOfRejectedNodes = 10000;
   private static final int maxNumberOfExpandedNodes = 10000;

   private static final int maxNumberOfChildNodes = 100;
   private static final double childNodeSize = 0.005;
   private static final double nodeSize = 0.007;

   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
   private final YoDouble time = new YoDouble("time", registry);

   private final YoInteger numberOfExpandedNodes = new YoInteger("expandedNodes", registry);
   private final YoInteger numberOfRejectedNodes = new YoInteger("rejectedNodes", registry);


   private final HashMap<QuadrupedFootstepPlannerNodeRejectionReason, List<FootstepNode>> rejectedNodes = new HashMap<>();
   private final Set<FootstepNode> expandedNodes = new HashSet<>();
   private final Set<FootstepNode> allRejectedNodes = new HashSet<>();
   private final HashMap<FootstepNode, List<FootstepNode>> childNodeMap = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> validChildNodeMap = new HashMap<>();
   private final HashMap<FootstepNode, List<FootstepNode>> rejectedChildNodeMap = new HashMap<>();



   private final List<YoFramePoint2D> yoActiveChildNodes = new ArrayList<>();
   private final List<YoFramePoint2D> yoActiveValidChildNodes = new ArrayList<>();
   private final List<YoFramePoint2D> yoActiveRejectedChildNodes = new ArrayList<>();

   private final List<YoFramePoint2D> yoExpandedNodes = new ArrayList<>();
   private final List<YoFramePoint2D> yoRejectedNodes = new ArrayList<>();

   private final YoGraphicsList rejectedNodesVis = new YoGraphicsList("plannerListener");
   private final YoGraphicsList expandedNodesVis = new YoGraphicsList("plannerListener");

   private final YoFootstepNode currentFootstepNode = new YoFootstepNode(registry);

   private final YoBoolean viewAllExpandedNodes = new YoBoolean("viewAllExpandedNodes", registry);
   private final YoBoolean viewAllRejectedNodes = new YoBoolean("viewAllRejectedNodes", registry);


   public QuadrupedAStarFootstepPlannerVisualizer(PlanarRegionsList regions)
   {
      this(0.1, regions);
   }

   public QuadrupedAStarFootstepPlannerVisualizer(double playbackRate, PlanarRegionsList regions)
   {
      for (QuadrupedFootstepPlannerNodeRejectionReason reasons : QuadrupedFootstepPlannerNodeRejectionReason.values)
      {
         rejectedNodes.put(reasons, new ArrayList<>());
      }

      viewAllExpandedNodes.set(true);
      viewAllRejectedNodes.set(true);

      for (int i = 0; i < maxNumberOfChildNodes; i++)
      {
         YoFramePoint2D activeChildNode = new YoFramePoint2D("activeChildNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoFramePoint2D activeValidChildNode = new YoFramePoint2D("activeValidChildNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoFramePoint2D activeRejectedChildNode = new YoFramePoint2D("activeRejectedChildNode" + i, ReferenceFrame.getWorldFrame(), registry);

         yoActiveChildNodes.add(activeChildNode);
         yoActiveValidChildNodes.add(activeValidChildNode);
         yoActiveRejectedChildNodes.add(activeRejectedChildNode);

         YoGraphicPosition activeChildNodeVis = new YoGraphicPosition("activeChildNode" + i, activeChildNode, childNodeSize, YoAppearance.Green());
         YoGraphicPosition activeValidChildNodeVis = new YoGraphicPosition("activeValidChildNode" + i, activeValidChildNode, childNodeSize, YoAppearance.Blue());
         YoGraphicPosition activeRejectedChildNodeVis = new YoGraphicPosition("activeRejectedChildNode" + i, activeRejectedChildNode, childNodeSize, YoAppearance.Red());

         graphicsListRegistry.registerYoGraphic("plannerListener", activeChildNodeVis);
         graphicsListRegistry.registerYoGraphic("plannerListener", activeValidChildNodeVis);
         graphicsListRegistry.registerYoGraphic("plannerListener", activeRejectedChildNodeVis);
      }

      for (int i = 0; i < maxNumberOfExpandedNodes; i++)
      {
         YoFramePoint2D expandedNode = new YoFramePoint2D("expandedNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition expandedNodeVis = new YoGraphicPosition("expandedNode" + i, expandedNode, childNodeSize, YoAppearance.Green());

         expandedNodesVis.add(expandedNodeVis);
         yoExpandedNodes.add(expandedNode);
      }

      for (int i = 0; i < maxNumberOfRejectedNodes; i++)
      {
         YoFramePoint2D rejectedNode = new YoFramePoint2D("rejectedNode" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition rejectedNodeVis = new YoGraphicPosition("rejectedNode" + i, rejectedNode, childNodeSize, YoAppearance.Black());

         rejectedNodesVis.add(rejectedNodeVis);
         yoRejectedNodes.add(rejectedNode);
      }

      graphicsListRegistry.registerYoGraphicsList(expandedNodesVis);
      graphicsListRegistry.registerYoGraphicsList(rejectedNodesVis);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoGraphicPosition otherFeet = new YoGraphicPosition(robotQuadrant.getShortName() + "activeNodeFeet", currentFootstepNode.getYoPosition(robotQuadrant), nodeSize, YoAppearance.OrangeRed());
         graphicsListRegistry.registerYoGraphic("plannerListener", otherFeet);

      }
      YoGraphicPosition nodeVis = new YoGraphicPosition("activeNode", currentFootstepNode.getMovingYoPosition(), nodeSize, YoAppearance.Orange());

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
      scs.tickAndUpdate();

   }

   @Override
   public void addNode(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null)
         return;

      if (!childNodeMap.containsKey(previousNode))
         childNodeMap.put(previousNode, new ArrayList<>());
      if (!validChildNodeMap.containsKey(previousNode))
         validChildNodeMap.put(previousNode, new ArrayList<>());

      childNodeMap.get(previousNode).add(node);
      validChildNodeMap.get(previousNode).add(node);

      currentFootstepNode.set(previousNode);

      expandedNodes.add(previousNode);

      numberOfExpandedNodes.set(expandedNodes.size());
   }

   @Override
   public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason reason)
   {
      validChildNodeMap.get(parentNode).remove(rejectedNode);

      if (!rejectedChildNodeMap.containsKey(parentNode))
         rejectedChildNodeMap.put(parentNode, new ArrayList<>());

      rejectedChildNodeMap.get(parentNode).add(rejectedNode);
      allRejectedNodes.add(rejectedNode);

      numberOfRejectedNodes.set(allRejectedNodes.size());
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
      yoActiveChildNodes.forEach(YoFramePoint2D::setToNaN);
      yoActiveValidChildNodes.forEach(YoFramePoint2D::setToNaN);
      yoActiveRejectedChildNodes.forEach(YoFramePoint2D::setToNaN);

      yoRejectedNodes.forEach(YoFramePoint2D::setToNaN);
      yoExpandedNodes.forEach(YoFramePoint2D::setToNaN);

      FootstepNode currentNode = currentFootstepNode.getEquivalentNode();

      if (currentNode == null)
         return;

      List<FootstepNode> childNodes = childNodeMap.get(currentNode);
      if (childNodes != null)
      {
         for (int i = 0; i < childNodes.size(); i++)
         {
            FootstepNode childNode = childNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoActiveChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }

      List<FootstepNode> validChildNodes = validChildNodeMap.get(currentNode);
      if (validChildNodes != null)
      {
         for (int i = 0; i < validChildNodes.size(); i++)
         {
            FootstepNode childNode = validChildNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoActiveValidChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }


      List<FootstepNode> rejectedChildNodes = rejectedChildNodeMap.get(currentNode);
      if (rejectedChildNodes != null)
      {
         for (int i = 0; i < rejectedChildNodes.size(); i++)
         {
            FootstepNode childNode = rejectedChildNodes.get(i);
            RobotQuadrant quadrant = childNode.getMovingQuadrant();
            yoActiveRejectedChildNodes.get(i).set(childNode.getX(quadrant), childNode.getY(quadrant));
         }
      }

      rejectedNodesVis.setVisible(viewAllRejectedNodes.getBooleanValue());
      expandedNodesVis.setVisible(viewAllExpandedNodes.getBooleanValue());

      int nodeNumber = 0;
      for (FootstepNode expandedNode : expandedNodes)
      {
         RobotQuadrant quadrant = expandedNode.getMovingQuadrant();
         yoExpandedNodes.get(nodeNumber).set(expandedNode.getX(quadrant), expandedNode.getY(quadrant));
         nodeNumber++;
      }

      nodeNumber = 0;
      for (FootstepNode rejectedNode : allRejectedNodes)
      {
         RobotQuadrant quadrant = rejectedNode.getMovingQuadrant();
         yoRejectedNodes.get(nodeNumber).set(rejectedNode.getX(quadrant), rejectedNode.getY(quadrant));
         nodeNumber++;
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
