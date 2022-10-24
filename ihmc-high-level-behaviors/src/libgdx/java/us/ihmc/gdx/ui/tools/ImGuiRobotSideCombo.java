package us.ihmc.gdx.ui.tools;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.robotics.robotSide.RobotSide;

public class ImGuiRobotSideCombo
{
   private final ImInt sideIndex = new ImInt();
   private String[] sideNames = new String[] {"Left", "Right"};

   public boolean combo(String label)
   {
      return ImGui.combo(label, sideIndex, sideNames);
   }

   public RobotSide getSide()
   {
      return RobotSide.values[sideIndex.get()];
   }

   public void setSide(RobotSide side)
   {
      sideIndex.set(side.ordinal());
   }
}
