package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.ros2.tf2.ROS2MutableFrame;
import us.ihmc.communication.ros2.tf2.ROS2StaticFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class RDXROS2TF2Demo
{
   private final Map<ROS2MutableFrame, RDXPose3DGizmo> mutableFrameMap = new HashMap<>();
   private final Map<ROS2StaticFrame, RDXReferenceFrameGraphic> staticFrameMap = new HashMap<>();
   private final Map<String, ReferenceFrame> allFrames = new TreeMap<>();
   private String[] frameIds;

   private final RDXBaseUI baseUI = new RDXBaseUI();

   private final ImString frameIdToAdd = new ImString();
   private final ImInt parentFrameIdIndex = new ImInt();

   private final float[] transformYawPitchRoll = new float[3];
   private final float[] transformTranslation = new float[3];

   private final Throttler updateThrottler = new Throttler().setFrequency(60.0);

   private final AtomicBoolean destroyed = new AtomicBoolean(false);

   public RDXROS2TF2Demo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "TF2DemoDestroy"));

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            addFrame(ReferenceFrameTools.getWorldFrame());
            addFrame(new ROS2StaticFrame("map", ReferenceFrame.getWorldFrame(), new RigidBodyTransform()));

            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);

            baseUI.create();
         }

         @Override
         public void render()
         {
            update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderSettings()
         {
            ImGui.inputText("Frame ID", frameIdToAdd);
            ImGui.combo("Parent Frame", parentFrameIdIndex, frameIds);
            ImGui.inputFloat3("Yaw Pitch Roll", transformYawPitchRoll);
            ImGui.inputFloat3("Translation", transformTranslation);

            ImGui.beginDisabled(allFrames.containsKey(frameIdToAdd.get()));

            if (ImGui.button("Add as mutable frame"))
            {
               ReferenceFrame parentFrame = allFrames.get(frameIds[parentFrameIdIndex.get()]);
               RigidBodyTransform initialOffset = new RigidBodyTransform(new YawPitchRoll(transformYawPitchRoll[0],
                                                                                          transformYawPitchRoll[1],
                                                                                          transformYawPitchRoll[2]),
                                                                         new Vector3D(transformTranslation[0],
                                                                                      transformTranslation[1],
                                                                                      transformTranslation[2]));
               ROS2MutableFrame mutableFrame = new ROS2MutableFrame(frameIdToAdd.get(), parentFrame, initialOffset);
               addFrame(mutableFrame);
            }

            ImGui.sameLine();
            if (ImGui.button("Add as static frame"))
            {
               ReferenceFrame parentFrame = allFrames.get(frameIds[parentFrameIdIndex.get()]);
               RigidBodyTransform initialOffset = new RigidBodyTransform(new YawPitchRoll(transformYawPitchRoll[0],
                                                                                          transformYawPitchRoll[1],
                                                                                          transformYawPitchRoll[2]),
                                                                         new Vector3D(transformTranslation[0],
                                                                                      transformTranslation[1],
                                                                                      transformTranslation[2]));
               ROS2StaticFrame staticFrame = new ROS2StaticFrame(frameIdToAdd.get(), parentFrame, initialOffset);
               addFrame(staticFrame);
            }

            ImGui.endDisabled();
         }

         @Override
         public void dispose()
         {
            staticFrameMap.values().forEach(RDXReferenceFrameGraphic::dispose);
            baseUI.dispose();
            destroy();
         }
      });
   }

   private void addFrame(ReferenceFrame frame)
   {
      allFrames.put(frame.getName(), frame);
      frameIds = new String[allFrames.size()];
      int i = 0;
      for (String id : allFrames.keySet())
      {
         frameIds[i++] = id;
      }

      if (frame instanceof ROS2StaticFrame staticFrame)
      {
         RDXReferenceFrameGraphic referenceFrameGraphic = new RDXReferenceFrameGraphic(0.3, Color.RED);
         referenceFrameGraphic.setToReferenceFrame(staticFrame);
         baseUI.getPrimaryScene().addRenderableProvider(referenceFrameGraphic);
         staticFrameMap.put(staticFrame, referenceFrameGraphic);
      }
      else if (frame instanceof ROS2MutableFrame mutableFrame)
      {
         RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo(mutableFrame, mutableFrame.getTransformToParent());
         poseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
         baseUI.getPrimaryScene().addRenderableProvider(poseGizmo);
         mutableFrameMap.put(mutableFrame, poseGizmo);
      }
   }

   private void update()
   {
      // Update mutable reference frames from the graphics
      mutableFrameMap.forEach((frame, gizmo) ->
      {
         if (!frame.getTransformToParent().geometricallyEquals(gizmo.getTransformToParent(), 1E-4))
         {
            frame.setNewTransformToParent(gizmo.getTransformToParent());
         }
      });

      // Update static frame graphics from the frame
      staticFrameMap.values().forEach(RDXReferenceFrameGraphic::updateFromLastGivenFrame);

      // Update all frames
      if (updateThrottler.run()) // TODO: remove update throttler
         allFrames.values().forEach(ReferenceFrame::update);
   }

   private void destroy()
   {
      if (destroyed.getAndSet(false))
      {
         allFrames.values().forEach(ReferenceFrame::remove);
      }
   }

   public static void main(String[] args)
   {
      new RDXROS2TF2Demo();
   }
}
