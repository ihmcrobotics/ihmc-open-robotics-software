package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.DoorParameterPacket;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

public class DetectedObjectViewer
{
   private final Group root = new Group();
   protected final ObservableList<Node> children = root.getChildren();

   private final AtomicReference<MeshView> meshToRender = new AtomicReference<>(null);

   private final AtomicReference<Boolean> clear = new AtomicReference<>(false);

   private static final double doorEdgeVizThickness = 0.01;
   private static final double doorHingeVizRadius = 0.03;
   private static final Color doorEdgeColor = Color.PINK;
   private static final Color doorHingeColor = Color.GREEN;

   public DetectedObjectViewer(Ros2Node ros2Node)
   {
      String doorParameterPacketTopicName = ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(DoorParameterPacket.class);
      ROS2Tools.createCallbackSubscription(ros2Node, DoorParameterPacket.class, doorParameterPacketTopicName, this::renderDoor);
   }

   public void renderDoor(Subscriber<DoorParameterPacket> subscriber)
   {
      DoorParameterPacket message = subscriber.takeNextData();
      double doorHeight = message.getDoorHeight();
      Point3D hingePoint = message.getHingedPointOnGround();
      Point3D endPoint = message.getEndPointOnGround();

      LogTools.info("doorHeight " + doorHeight);
      LogTools.info("hingePoint " + hingePoint);
      LogTools.info("endPoint " + endPoint);

      List<Point3D> doorVertices = new ArrayList<Point3D>();
      doorVertices.add(hingePoint);
      doorVertices.add(new Point3D(hingePoint.getX(), hingePoint.getY(), hingePoint.getZ() + doorHeight));
      doorVertices.add(new Point3D(endPoint.getX(), endPoint.getY(), endPoint.getZ() + doorHeight));
      doorVertices.add(endPoint);

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();
      meshBuilder.addMultiLine(doorVertices, doorEdgeVizThickness, doorEdgeColor, true);
      meshBuilder.addSphere(doorHingeVizRadius, hingePoint, doorHingeColor);
      MeshView doorMeshView = new MeshView(meshBuilder.generateMesh());
      doorMeshView.setMaterial(meshBuilder.generateMaterial());
      meshToRender.set(doorMeshView);
   }

   public void render()
   {
      MeshView newScanMeshView = meshToRender.getAndSet(null);

      if (clear.getAndSet(false))
         children.clear();

      if (newScanMeshView != null)
      {
         children.add(newScanMeshView);
      }
   }

   public void clear()
   {
      clear.set(true);
   }

   public Node getRoot()
   {
      return root;
   }
}
