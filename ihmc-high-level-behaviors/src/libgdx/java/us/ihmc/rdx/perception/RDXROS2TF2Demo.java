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

public class RDXROS2TF2Demo
{
   private final Map<ROS2MutableFrame, RDXPose3DGizmo> mutableFrameMap = new HashMap<>();
   private final Map<ROS2StaticFrame, RDXReferenceFrameGraphic> staticFrameMap = new HashMap<>();
   private final Map<String, ReferenceFrame> allFrames = new TreeMap<>();
   private String[] frameIds;

   private final RDXBaseUI baseUI = new RDXBaseUI();

   private final ImString frameIdToAdd = new ImString();
   private final ImInt parentFrameIdIndex = new ImInt();
   private final ImInt frameToRemoveIndex = new  ImInt();

   private final float[] transformYawPitchRoll = new float[3];
   private final float[] transformTranslation = new float[3];

   private final Throttler updateThrottler = new Throttler().setFrequency(60.0);

   private RDXROS2TF2Demo()
   {
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

            ImGui.separator();

            ImGui.combo("###FrameToRemove", frameToRemoveIndex, frameIds);
            ImGui.sameLine();
            if (ImGui.button("Remove"))
               removeFrame(allFrames.get(frameIds[frameToRemoveIndex.get()]));
         }

         @Override
         public void dispose()
         {
            staticFrameMap.values().forEach(RDXReferenceFrameGraphic::dispose);
            baseUI.dispose();
         }
      });
   }

   private void updateFrameIds()
   {
      frameIds = new String[allFrames.size()];
      int i = 0;
      for (CharSequence id : allFrames.keySet())
      {
         frameIds[i++] = id.toString();
      }
   }

   private void addFrame(ReferenceFrame frame)
   {
      allFrames.put(frame.getName(), frame);

      updateFrameIds();

      if (frame instanceof ROS2StaticFrame staticFrame)
      {
         RDXReferenceFrameGraphic referenceFrameGraphic = new RDXReferenceFrameGraphic(0.3, Color.RED);
         referenceFrameGraphic.setToReferenceFrame(staticFrame);
         baseUI.getPrimaryScene().addRenderableProvider(referenceFrameGraphic, referenceFrameGraphic);
         staticFrameMap.put(staticFrame, referenceFrameGraphic);
      }
      else if (frame instanceof ROS2MutableFrame mutableFrame)
      {
         RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo(mutableFrame, mutableFrame.getTransformToParent());
         poseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
         mutableFrameMap.put(mutableFrame, poseGizmo);
      }
   }

   private void removeFrame(ReferenceFrame frameToRemove)
   {
      allFrames.remove(frameToRemove.getName());

      if (frameToRemove instanceof ROS2MutableFrame mutableFrame)
      {
         RDXPose3DGizmo gizmo = mutableFrameMap.remove(mutableFrame);
         if (gizmo != null)
         {
            baseUI.getPrimaryScene().removeRenderable(gizmo);
            gizmo.destroyDefault(baseUI.getPrimary3DPanel());
         }
      }
      else if (frameToRemove instanceof ROS2StaticFrame staticFrame)
      {
         RDXReferenceFrameGraphic graphic = staticFrameMap.remove(staticFrame);
         baseUI.getPrimaryScene().removeRenderable(graphic);
         if (graphic != null)
            graphic.dispose();
      }
      frameToRemove.remove();

      updateFrameIds();
   }

   private void update()
   {
      // Update all frames
      if (updateThrottler.run()) // TODO: remove update throttler
         allFrames.values().forEach(ReferenceFrame::update);

      // Update mutable reference frames from the graphics
      mutableFrameMap.forEach((frame, gizmo) ->
      {
         if (gizmo.isBeingManipulated() && !frame.getTransformToParent().geometricallyEquals(gizmo.getTransformToParent(), 1E-4))
         {
            frame.setNewTransformToParent(gizmo.getTransformToParent());
         }
      });

      // Update static frame graphics from the frame
      staticFrameMap.values().forEach(RDXReferenceFrameGraphic::updateFromLastGivenFrame);
   }

   public static void main(String[] args)
   {
      new RDXROS2TF2Demo();
   }
}
