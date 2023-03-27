package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.objects.ArUcoMarkerObject;
import us.ihmc.perception.objects.ArUcoMarkerObjectInfo;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXVirtualGhostObject;
import us.ihmc.ros2.ROS2Topic;

public class RDXArUcoObjectVisualizer
{
   private final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("object_detector");
   public final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = BASE_TOPIC.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final ROS2Helper ros2;
   private boolean objectDetected = false;
   private ArUcoMarkerObject objectWithArUcoMarker;
   private ArUcoMarkerObjectInfo objectInfo;
   private String objectName = "";

   private final ImGuiPanel panel = new ImGuiPanel("ArUco Object Visualizer", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final ImBoolean showGraphics = new ImBoolean(false);
   private RDXVirtualGhostObject objectBody;
   private RDXVirtualGhostObject objectAppendix;


   public RDXArUcoObjectVisualizer(ArUcoMarkerObjectInfo objectInfo)
   {
      ros2 = new ROS2Helper(DomainFactory.PubSubImplementation.FAST_RTPS, "object_detector");
      arUcoMarkerPosesSubscription = ros2.subscribe(ARUCO_MARKER_POSES);
      this.objectInfo = objectInfo;
   }

   public void update()
   {
      if (enabled.get())
      {
         if (arUcoMarkerPosesSubscription.getMessageNotification().poll())
         {
            ArUcoMarkerPoses arUcoMarkerPosesMessage = arUcoMarkerPosesSubscription.getMessageNotification().read();
            if (arUcoMarkerPosesMessage.getMarkerId().size() > 0)
               objectDetected = true;
            else
               objectDetected = false;
            for (int i = 0; i < arUcoMarkerPosesMessage.getMarkerId().size(); i++)
            {
               int objectId = (int) arUcoMarkerPosesMessage.getMarkerId().get(i);
               objectName = objectInfo.getObjectName(objectId);
               objectWithArUcoMarker = new ArUcoMarkerObject(objectId, objectInfo);
               // get marker pose in camera fram
               FramePose3DBasics markerPose = new FramePose3D();
               markerPose.getPosition().set(arUcoMarkerPosesMessage.getPosition().get(i));
               markerPose.getOrientation().set(arUcoMarkerPosesMessage.getOrientation().get(i));
               // transform in world frame
               markerPose.changeFrame(ReferenceFrame.getWorldFrame());
               // set transform MarkerToWorld to markerPose
               markerPose.get(objectWithArUcoMarker.getMarkerTransformToWorld());
               // update frame of the object
               objectWithArUcoMarker.updateFrame();
               // compute object pose from marker pose
               objectWithArUcoMarker.computeObjectPose(markerPose);
            }
         }
         if (showGraphics.get())
            updateGraphics();
      }
   }

   private void updateGraphics()
   {
      if (hasDetectedObject() && objectBody == null)
      {
         objectName = getObjectName();
         objectBody = new RDXVirtualGhostObject(objectInfo.getVirtualBodyFileName(objectName));
         if (objectInfo.hasAppendix(objectName))
            objectAppendix = new RDXVirtualGhostObject(objectInfo.getVirtualAppendixFileName(objectName));
      }
      if (hasDetectedObject() && objectBody != null)
      {
         RigidBodyTransform transformObjectToWorld = new RigidBodyTransform();
         getObjectPose().get(transformObjectToWorld);
         objectBody.setTransformToParent(transformObjectToWorld);
         objectBody.update();
         if (objectAppendix != null)
         {
            RigidBodyTransform transformAppendixToWorld = new RigidBodyTransform();
            getObjectAppendixPose().get(transformAppendixToWorld);
            objectBody.setTransformToParent(transformAppendixToWorld);
            objectAppendix.setTransformToParent(transformAppendixToWorld);
            objectAppendix.update();
         }
      }
      else
      {
         objectBody = null;
         objectAppendix = null;
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Enabled"), enabled);
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGraphics.get())
      {
         if (objectBody != null)
            objectBody.getRenderables(renderables,pool);
         if (objectAppendix != null)
            objectAppendix.getRenderables(renderables,pool);
      }
   }

   public String getObjectName()
   {
      return objectName;
   }

   public FramePose3D getObjectPose()
   {
      return objectWithArUcoMarker.getObjectPose();
   }

   public FramePose3D getObjectAppendixPose()
   {
      return objectWithArUcoMarker.getAppendixPose();
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectWithArUcoMarker.getObjectFrame();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public boolean isEnabled()
   {
      return enabled.get();
   }

   public void setEnabled(boolean enable)
   {
      this.enabled.set(enable);
   }

   public boolean hasDetectedObject()
   {
      return objectDetected;
   }
}
