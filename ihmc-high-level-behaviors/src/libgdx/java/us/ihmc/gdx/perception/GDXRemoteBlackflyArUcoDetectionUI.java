package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.ArUcoMarkerPoses;
import std_msgs.msg.dds.Float64;
import us.ihmc.avatar.colorVision.DualBlackflyComms;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;

public class GDXRemoteBlackflyArUcoDetectionUI
{
   private final ImGuiPanel panel = new ImGuiPanel("ArUco Marker Detection", this::renderImGuiWidgets);
   private final SideDependentList<IHMCROS2Input<Float64>> publishRateInputs = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlot> publishRatePlots = new SideDependentList<>();
   private final IHMCROS2Input<Float64> getImageDurationInput;
   private final ImPlotDoublePlot getImageDurationPlot;
   private final IHMCROS2Input<Float64> convertColorDurationInput;
   private final ImPlotDoublePlot convertColorDurationPlot;
   private final IHMCROS2Input<Float64> encodingDurationInput;
   private final ImPlotDoublePlot encodingDurationPlot;
   private final IHMCROS2Input<Float64> copyDurationInput;
   private final ImPlotDoublePlot copyDurationPlot;
   private final TypedNotification<ArUcoMarkerPoses> arUcoPoseROS2Notification = new TypedNotification<>();
   private final HashMap<Long, GDXModelInstance> markerPoseCoordinateFrames = new HashMap<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public GDXRemoteBlackflyArUcoDetectionUI(ROS2Helper ros2Helper)
   {
      for (RobotSide side : RobotSide.values)
      {
         publishRateInputs.put(side, ros2Helper.subscribe(DualBlackflyComms.PUBLISH_RATE.get(side)));
         publishRatePlots.put(side, new ImPlotDoublePlot(side.getPascalCaseName() + " Publish Rate", 30));
      }
      getImageDurationInput = ros2Helper.subscribe(DualBlackflyComms.GET_IMAGE_DURATION);
      getImageDurationPlot = new ImPlotDoublePlot("Get image duration", 30);
      convertColorDurationInput = ros2Helper.subscribe(DualBlackflyComms.CONVERT_COLOR_DURATION);
      convertColorDurationPlot = new ImPlotDoublePlot("Convert color duration", 30);
      encodingDurationInput = ros2Helper.subscribe(DualBlackflyComms.ENCODING_DURATION);
      encodingDurationPlot = new ImPlotDoublePlot("Encoding duration", 30);
      copyDurationInput = ros2Helper.subscribe(DualBlackflyComms.COPY_DURATION);
      copyDurationPlot = new ImPlotDoublePlot("Copy duration", 30);

      ros2Helper.subscribeViaCallback(DualBlackflyComms.FRAME_POSE, arUcoPoseROS2Notification::set);
   }

   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (publishRateInputs.get(side).hasReceivedFirstMessage())
         {
            publishRatePlots.get(side).addValue(publishRateInputs.get(side).getLatest().getData());
         }
         getImageDurationPlot.addValue(getImageDurationInput.getLatest().getData());
         convertColorDurationPlot.addValue(convertColorDurationInput.getLatest().getData());
         encodingDurationPlot.addValue(encodingDurationInput.getLatest().getData());
         copyDurationPlot.addValue(copyDurationInput.getLatest().getData());
      }

      ArUcoMarkerPoses markerPoses;
      if (arUcoPoseROS2Notification.poll())
      {
         markerPoses = arUcoPoseROS2Notification.read();

         for (int i = 0; i < markerPoses.getMarkerId().size(); i++)
         {
            long markerID = markerPoses.getMarkerId().get(i);
            GDXModelInstance markerPoseFrame = markerPoseCoordinateFrames.get(markerID);
            if (markerPoseFrame == null)
            {
               markerPoseFrame = new GDXModelInstance(GDXModelBuilder.createCoordinateFrame(0.4));
               markerPoseCoordinateFrames.put(markerID, markerPoseFrame);
            }

            tempTransform.set(markerPoses.getOrientation().get(i), markerPoses.getPosition().get(i));
            markerPoseFrame.setTransformToWorldFrame(tempTransform);
         }
      }
   }

   public void renderImGuiWidgets()
   {
      for (RobotSide side : RobotSide.values)
      {
         publishRatePlots.get(side).renderImGuiWidgets();
      }
      getImageDurationPlot.renderImGuiWidgets();
      convertColorDurationPlot.renderImGuiWidgets();
      encodingDurationPlot.renderImGuiWidgets();
      copyDurationPlot.renderImGuiWidgets();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXModelInstance markerPoseCoordinateFrame : markerPoseCoordinateFrames.values())
      {
         markerPoseCoordinateFrame.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      for (RobotSide side : RobotSide.values)
      {
         publishRateInputs.get(side).destroy();
      }
      getImageDurationInput.destroy();
      convertColorDurationInput.destroy();
      encodingDurationInput.destroy();
      copyDurationInput.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
