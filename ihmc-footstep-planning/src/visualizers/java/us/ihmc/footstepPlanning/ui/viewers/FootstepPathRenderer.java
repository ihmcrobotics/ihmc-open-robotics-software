package us.ihmc.footstepPlanning.ui.viewers;

import controller_msgs.msg.dds.FootstepNodeDataListMessage;
import controller_msgs.msg.dds.FootstepNodeDataMessage;
import javafx.scene.Group;
import javafx.scene.Node;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.idl.IDLSequence;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.FootstepPlanTopic;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.NodeDataTopic;

public class FootstepPathRenderer
{
   private final Group root = new Group();
   private final FootstepPathMeshViewer footstepPathMeshViewer = new FootstepPathMeshViewer();

   public FootstepPathRenderer(Messager messager)
   {
      messager.registerTopicListener(NodeDataTopic, this::processLowestCostNodeList);
      messager.registerTopicListener(FootstepPlanTopic, footstepPathMeshViewer::processFootstepPath);
      root.getChildren().add(footstepPathMeshViewer.getRoot());
   }

   private void processLowestCostNodeList(FootstepNodeDataListMessage message)
   {
      if(message.getIsFootstepGraph())
         return;

      IDLSequence.Object<FootstepNodeDataMessage> nodeDataList = message.getNodeData();
      FootstepPlan footstepPlan = new FootstepPlan();
      for (int i = 0; i < nodeDataList.size(); i++)
      {
         addNodeDataToFootstepPlan(footstepPlan, nodeDataList.get(i));
      }

      footstepPathMeshViewer.processFootstepPath(footstepPlan);
   }

   private static void addNodeDataToFootstepPlan(FootstepPlan footstepPlan, FootstepNodeDataMessage nodeData)
   {
      RobotSide robotSide = RobotSide.fromByte(nodeData.getRobotSide());

      RigidBodyTransform footstepPose = new RigidBodyTransform();
      footstepPose.setRotationYawAndZeroTranslation(nodeData.getYawIndex() * FootstepNode.gridSizeYaw);
      footstepPose.setTranslationX(nodeData.getXIndex() * FootstepNode.gridSizeXY);
      footstepPose.setTranslationY(nodeData.getYIndex() * FootstepNode.gridSizeXY);

      RigidBodyTransform snapTransform = new RigidBodyTransform();
      snapTransform.set(nodeData.getSnapRotation(), nodeData.getSnapTranslation());
      snapTransform.transform(footstepPose);
      footstepPlan.addFootstep(robotSide, new FramePose3D(ReferenceFrame.getWorldFrame(), footstepPose));
   }

   public void start()
   {
      footstepPathMeshViewer.start();
   }

   public void stop()
   {
      footstepPathMeshViewer.stop();
   }

   public Node getRoot()
   {
      return root;
   }
}
