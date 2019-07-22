package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.visualization;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class YoFootstepNode
{
   private final YoEnum<RobotQuadrant> movingQuadrant;
   private final YoFramePoint2D movingNodePosition;
   private final QuadrantDependentList<YoFramePoint2D> nodePositions = new QuadrantDependentList<>();
   private final YoDouble nominalStanceLength;
   private final YoDouble nominalStanceWidth;
   private final YoDouble nodeYaw;

   public YoFootstepNode(YoVariableRegistry registry)
   {
      movingQuadrant = YoEnum.create("movingQuadrant", RobotQuadrant.class, registry);
      movingNodePosition = new YoFramePoint2D("movingNodePosition", ReferenceFrame.getWorldFrame(), registry);
      nodeYaw = new YoDouble("nodeYaw", registry);
      movingNodePosition.setToNaN();
      nominalStanceLength = new YoDouble("nominalStanceLength", registry);
      nominalStanceWidth = new YoDouble("nominalStanceWidth", registry);
      nominalStanceLength.setToNaN();
      nominalStanceWidth.setToNaN();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint2D nodePosition =  new YoFramePoint2D(robotQuadrant.getShortName() + "NodePosition", ReferenceFrame.getWorldFrame(), registry);
         nodePosition.setToNaN();
         nodePositions.put(robotQuadrant, nodePosition);
      }
   }

   public void set(FootstepNode node)
   {
      movingQuadrant.set(node.getMovingQuadrant());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         nodePositions.get(robotQuadrant).set(node.getX(robotQuadrant), node.getY(robotQuadrant));
      movingNodePosition.set(nodePositions.get(movingQuadrant.getEnumValue()));
      nominalStanceLength.set(node.getNominalStanceLength());
      nominalStanceWidth.set(node.getNominalStanceWidth());
      nodeYaw.set(node.getStepYaw());
   }

   public YoFramePoint2D getYoPosition(RobotQuadrant robotQuadrant)
   {
      return nodePositions.get(robotQuadrant);
   }

   public YoFramePoint2D getMovingYoPosition()
   {
      return movingNodePosition;
   }

   public FootstepNode getEquivalentNode()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (nodePositions.get(robotQuadrant).containsNaN())
            return null;
      }

      return new FootstepNode(movingQuadrant.getEnumValue(), nodePositions.get(RobotQuadrant.FRONT_LEFT), nodePositions.get(RobotQuadrant.FRONT_RIGHT),
                              nodePositions.get(RobotQuadrant.HIND_LEFT), nodePositions.get(RobotQuadrant.HIND_RIGHT), nodeYaw.getDoubleValue(),
                              nominalStanceLength.getDoubleValue(), nominalStanceWidth.getDoubleValue());
   }
}
