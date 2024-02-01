package us.ihmc.rdx.perception;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SteppableRegionDebugImageMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.steppableRegions.SteppableRegionsAPI;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;

public class RDXSteppableRegionsPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean renderRegions = new ImBoolean(false);

   private final OpenCLManager openCLManager = new OpenCLManager();

   private final AtomicReference<SteppableRegionDebugImagesMessage> incomingSteppableRegionDebugImagesToRender = new AtomicReference<>();

   private final AtomicReference<SteppableRegionDebugImagesMessage> latestSteppableRegionDebugImagesToRender = new AtomicReference<>();
   private int latestIndexToRender = 0;
   private boolean shouldUpdateRender = false;

   private final ImBoolean regionsActive = new ImBoolean(false);
   private final ImBoolean drawPatches = new ImBoolean(true);
   private final ImInt yawIndexToRender = new ImInt(0);

   private RDXPanel imguiPanel;
   private RDXMatImagePanel steppabilityPanel;
   private RDXMatImagePanel steppableRegionsPanel;

   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;
   private int cellsPerSide;
   private final SteppableRegionCalculatorParameters parameters;
   private ROS2Heartbeat steppableRegionsHeartbeat;

   public RDXSteppableRegionsPanel(ROS2Helper ros2Helper, SteppableRegionCalculatorParametersReadOnly defaultParameters)
   {
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      parameters = new SteppableRegionCalculatorParameters(defaultParameters);
      remotePropertySets.registerRemotePropertySet(parameters, SteppableRegionsAPI.PARAMETERS);
   }

   public void create()
   {
      imguiPanel = new RDXPanel("Steppable Region Extraction", this::renderImGuiWidgetsPanel);

      cellsPerSide = 100;
      steppabilityPanel = new RDXMatImagePanel("Raw Steppability ", cellsPerSide, cellsPerSide, false);
      steppableRegionsPanel = new RDXMatImagePanel("Steppable Regions ", cellsPerSide, cellsPerSide, false);
      imguiPanel.addChild(steppabilityPanel.getImagePanel());
      imguiPanel.addChild(steppableRegionsPanel.getImagePanel());
   }

   volatile boolean needToDraw = false;

   public void setUpForNetworking(ROS2Node ros2Node)
   {
      new IHMCROS2Callback<>(ros2Node, SteppableRegionsAPI.STEPPABLE_REGIONS_DEBUG_OUTPUT, this::setLatestSteppableRegionDebugImagesToRender);
      steppableRegionsHeartbeat = new ROS2Heartbeat(ros2Node, SteppableRegionsAPI.PUBLISH_STEPPABLE_REGIONS);
   }

   public void setLatestSteppableRegionDebugImagesToRender(SteppableRegionDebugImagesMessage steppableRegionDebugImagesToRender)
   {
      this.incomingSteppableRegionDebugImagesToRender.set(steppableRegionDebugImagesToRender);
   }

   public void update()
   {
      if (steppableRegionsHeartbeat != null)
      {
         steppableRegionsHeartbeat.setAlive(regionsActive.get());
      }

      if (renderRegions.get())
      {
         updateValuesToRender();

         if (shouldUpdateRender)
         {
            drawDebugRegions();
         }

         if (needToDraw)
         {
            draw2DPanels();
         }
      }
   }

   private void resize(int newSize)
   {
      if (newSize != cellsPerSide)
      {
         cellsPerSide = newSize;
         steppableRegionsPanel.resize(newSize, newSize);
         steppabilityPanel.resize(newSize, newSize);
      }
   }

   private void updateValuesToRender()
   {
      SteppableRegionDebugImagesMessage debugImagesMessage = incomingSteppableRegionDebugImagesToRender.getAndSet(null);
      if (debugImagesMessage != null)
      {
         shouldUpdateRender = true;
         latestSteppableRegionDebugImagesToRender.set(debugImagesMessage);
      }

      if (latestIndexToRender != yawIndexToRender.get())
      {
         shouldUpdateRender = true;
         latestIndexToRender = yawIndexToRender.get();
      }
   }
   private void drawDebugRegions()
   {
      needToDraw = false;
      SteppableRegionDebugImagesMessage debugImagesMessage = latestSteppableRegionDebugImagesToRender.get();
      if (debugImagesMessage == null)
         return;

      resize(debugImagesMessage.getRegionImages().get(0).getImageWidth());
      int yawToDraw = latestIndexToRender;

      if (steppableRegionsPanel.getImagePanel().getIsShowing().get() && drawPatches.get())
      {
         needToDraw = true;
         Mat image = steppableRegionsPanel.getImage();

         SteppableRegionDebugImageMessage regionImage = debugImagesMessage.getRegionImages().get(yawToDraw);

         if (image.arrayHeight() != regionImage.getImageHeight() || image.arrayWidth() != regionImage.getImageWidth())
            throw new RuntimeException("Sizes don't match");

         for (int row = 0; row < regionImage.getImageHeight(); row++)
         {
            for (int col = 0; col < regionImage.getImageWidth(); col++)
            {
               BytePointer pointer = image.ptr(row, col);
               for (int j = 0; j < 3; j++)
               {
                  int index = row * regionImage.getImageWidth() + col;
                  int start = 3 * index;
                  pointer.put(j, regionImage.getData().get(start + j));
               }
            }
         }
      }

      if (steppabilityPanel.getImagePanel().getIsShowing().get() && drawPatches.get())
      {
         needToDraw = true;

         Mat image = steppabilityPanel.getImage();
         SteppableRegionDebugImageMessage steppabilityImage = debugImagesMessage.getSteppabilityImages().get(yawToDraw);

         for (int row = 0; row < steppabilityImage.getImageHeight(); row++)
         {
            for (int col = 0; col < steppabilityImage.getImageWidth(); col++)
            {
               BytePointer pointer = image.ptr(row, col);
               for (int j = 0; j < 3; j++)
               {
                  int index = row * steppabilityImage.getImageWidth() + col;
                  int start = 3 * index;
                  pointer.put(j, steppabilityImage.getData().get(start + j));
               }
            }
         }
      }
   }

   private void draw2DPanels()
   {
      steppableRegionsPanel.display();
      steppabilityPanel.display();
   }

   private void renderImGuiWidgetsPanel()
   {
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      remotePropertySets.renderImGuiWidgets();

      ImGui.text("Input height map dimensions: " + cellsPerSide + " x " + cellsPerSide);

      ImGui.checkbox(labels.get("Render regions"), renderRegions);

      ImGui.sliderInt("Yaw index to render", yawIndexToRender.getData(), 0, parameters.getYawDiscretizations() - 1);

      ImGui.checkbox("Steppable regions active", regionsActive);
   }

   public void renderImGuiWidgets()
   {
   }

   public void destroy()
   {
      if (steppableRegionsHeartbeat != null)
         steppableRegionsHeartbeat.destroy();
      openCLManager.destroy();
   }

   public RDXPanel getBasePanel()
   {
      return imguiPanel;
   }

   public ImBoolean getEnabled()
   {
      return renderRegions;
   }
}
