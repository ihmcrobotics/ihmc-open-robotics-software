package us.ihmc.rdx.ui.algorithmUI;

import imgui.ImGui;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class RDXTunableAlgorithm extends RDXPanel
{
   private final String title;

   protected final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   protected final ROS2PublishSubscribeAPI ros2;
   protected final StoredPropertySetBasics algorithmPropertySet;
   protected final StoredPropertySetROS2TopicPair propertySetTopicPair;
   protected ImGuiRemoteROS2StoredPropertySet imGuiPropertySet;

   public RDXTunableAlgorithm(StoredPropertySetBasics algorithmPropertySet, StoredPropertySetROS2TopicPair propertySetTopicPair, ROS2PublishSubscribeAPI ros2)
   {
      this(algorithmPropertySet.getTitle(), algorithmPropertySet, propertySetTopicPair, ros2);
   }

   public RDXTunableAlgorithm(String title,
                              StoredPropertySetBasics algorithmPropertySet,
                              StoredPropertySetROS2TopicPair propertySetTopicPair,
                              ROS2PublishSubscribeAPI ros2)
   {
      super(title);

      this.title = title;
      this.algorithmPropertySet = algorithmPropertySet;
      this.propertySetTopicPair = propertySetTopicPair;
      this.ros2 = ros2;

      setRenderMethod(this::renderSettingsPanel);
   }

   public void create()
   {
      if (!isCreated())
         imGuiPropertySet = new ImGuiRemoteROS2StoredPropertySet(ros2, algorithmPropertySet, propertySetTopicPair);
   }

   public void create(RDXStoredPropertySetTuner tunerUI)
   {
      if (!isCreated())
         imGuiPropertySet = new ImGuiRemoteROS2StoredPropertySet(ros2, algorithmPropertySet, propertySetTopicPair, tunerUI);
   }

   public boolean renderMenuEntry()
   {
      boolean buttonPressed = false;
      float preButtonCursorY = ImGui.getCursorPosY();
      if (ImGui.button(labels.get(getTitle()), -1.0f, 0.0f))
      {
         getIsShowing().set(!isShowing());
         buttonPressed = true;
      }
      float postButtonCursorY = ImGui.getCursorPosY();

      if (getIsShowing().get())
      {
         ImGui.setCursorPosY(preButtonCursorY + (ImGui.getTextLineHeight() / 2) - 2);
         ImGuiTools.rightAlignText(">> ");
         ImGui.setCursorPosY(postButtonCursorY);
      }

      return buttonPressed;
   }

   public void renderSettingsPanel()
   {
      if (imGuiPropertySet != null)
         imGuiPropertySet.renderImGuiWidgetsWithUpdateButton();
   }

   public boolean isCreated()
   {
      return imGuiPropertySet != null;
   }

   public boolean isShowing()
   {
      return getIsShowing().get();
   }

   public String getTitle()
   {
      return title;
   }
}
