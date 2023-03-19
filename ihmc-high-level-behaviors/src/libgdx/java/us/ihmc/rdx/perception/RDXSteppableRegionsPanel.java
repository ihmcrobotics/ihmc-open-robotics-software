package us.ihmc.rdx.perception;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SteppableRegionDebugImageMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegionsAPI;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegionsCalculationModule;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class RDXSteppableRegionsPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean renderRegions = new ImBoolean(false);

   private final OpenCLManager openCLManager = new OpenCLManager();

   private final AtomicReference<SteppableRegionDebugImagesMessage> latestSteppableRegionDebugImagesToRender = new AtomicReference<>();

   private final ImBoolean drawPatches = new ImBoolean(true);

   private ImGuiPanel imguiPanel;
   private final List<RDXMatImagePanel> steppabilityPanels = new ArrayList<>();
   private final List<RDXMatImagePanel> steppableRegionsPanels = new ArrayList<>();

   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;
   private int cellsPerSide;
   private final SteppableRegionCalculatorParameters parameters;

   public RDXSteppableRegionsPanel(ROS2Helper ros2Helper, SteppableRegionCalculatorParametersReadOnly defaultParameters)
   {
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      parameters = new SteppableRegionCalculatorParameters(defaultParameters);
      remotePropertySets.registerRemotePropertySet(parameters, SteppableRegionsAPI.PARAMETERS);
   }

   public void create()
   {
      imguiPanel = new ImGuiPanel("Steppable Region Extraction", this::renderImGuiWidgetsPanel);

      cellsPerSide = 100;
      for (int i = 0; i < parameters.getYawDiscretizations(); i++)
      {
         RDXMatImagePanel steppabilityPanel = new RDXMatImagePanel("Raw Steppability " + i, cellsPerSide, cellsPerSide, false);
         RDXMatImagePanel panel = new RDXMatImagePanel("Steppable Regions " + i, cellsPerSide, cellsPerSide, false);
         steppabilityPanels.add(steppabilityPanel);
         steppableRegionsPanels.add(panel);
         imguiPanel.addChild(steppabilityPanel.getImagePanel());
         imguiPanel.addChild(panel.getImagePanel());
      }
   }

   volatile boolean needToDraw = false;

   public void setLatestSteppableRegionDebugImagesToRender(SteppableRegionDebugImagesMessage steppableRegionDebugImagesToRender)
   {
      this.latestSteppableRegionDebugImagesToRender.set(steppableRegionDebugImagesToRender);
   }

   public void update()
   {
      if (renderRegions.get())
      {
         drawDebugRegions();

         if (needToDraw)
         {
            draw2DPanels();

            needToDraw = false;
         }
      }
   }

   private void resize(int newSize)
   {
      if (newSize != cellsPerSide)
      {
         cellsPerSide = newSize;
         for (int i = 0; i < parameters.getYawDiscretizations(); i++)
         {
            steppableRegionsPanels.get(i).resize(newSize, newSize);
            steppabilityPanels.get(i).resize(newSize, newSize);
         }
      }
   }

   private void drawDebugRegions()
   {
      SteppableRegionDebugImagesMessage debugImagesMessage = latestSteppableRegionDebugImagesToRender.getAndSet(null);
      if (debugImagesMessage == null)
         return;

      resize(debugImagesMessage.getRegionImages().get(0).getImageWidth());

      for (int yaw = 0; yaw < parameters.getYawDiscretizations(); yaw++)
      {
         RDXMatImagePanel panel = steppableRegionsPanels.get(yaw);
         if (panel.getImagePanel().getIsShowing().get() && drawPatches.get())
         {
            Mat image = panel.getImage();

            SteppableRegionDebugImageMessage regionImage = debugImagesMessage.getRegionImages().get(yaw);

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

         panel = steppabilityPanels.get(yaw);
         if (panel.getImagePanel().getIsShowing().get() && drawPatches.get())
         {
            Mat image = panel.getImage();
            SteppableRegionDebugImageMessage steppabilityImage = debugImagesMessage.getSteppabilityImages().get(yaw);

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

      needToDraw = true;
   }

   private void draw2DPanels()
   {
      steppableRegionsPanels.forEach(RDXMatImagePanel::display);
      steppabilityPanels.forEach(RDXMatImagePanel::display);
   }

   private void renderImGuiWidgetsPanel()
   {
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      remotePropertySets.renderImGuiWidgets();

      ImGui.text("Input height map dimensions: " + cellsPerSide + " x " + cellsPerSide);

      ImGui.checkbox(labels.get("Render regions"), renderRegions);
   }

   public void renderImGuiWidgets()
   {
   }

   public void destroy()
   {
      openCLManager.destroy();
   }

   public ImGuiPanel getBasePanel()
   {
      return imguiPanel;
   }

   public ImBoolean getEnabled()
   {
      return renderRegions;
   }
}
