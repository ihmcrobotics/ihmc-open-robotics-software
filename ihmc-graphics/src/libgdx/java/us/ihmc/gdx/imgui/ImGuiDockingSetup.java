package us.ihmc.gdx.imgui;

import imgui.internal.ImGui;
import imgui.type.ImInt;

import java.util.ArrayList;
import java.util.HashMap;

public class ImGuiDockingSetup
{
   private final ArrayList<ImGuiDockingSetupInstruction> instructions = new ArrayList<>();
   private final HashMap<String, Integer> windowsSpaceIds = new HashMap<>();
   private final ArrayList<ImGuiWindow> windows = new ArrayList<>();

   public void addFirst(String windowName)
   {
      addFirst(new ImGuiWindow(windowName));
   }

   public void addFirst(ImGuiWindow window)
   {
      addInstruction(new ImGuiDockingSetupInstruction(window.getWindowName()), window);
   }

   public void splitAdd(String windowToAddName, int imGuiDir, double percent)
   {
      splitAdd(new ImGuiWindow(windowToAddName), imGuiDir, percent);
   }

   public void splitAdd(ImGuiWindow windowToAdd, int imGuiDir, double percent)
   {
      splitAdd(windows.get(windows.size() - 1).getWindowName(), windowToAdd, imGuiDir, percent);
   }

   public void splitAdd(String windowToSplitName, String windowToAddName, int imGuiDir, double percent)
   {
      splitAdd(windowToSplitName, new ImGuiWindow(windowToAddName), imGuiDir, percent);
   }

   public void splitAdd(String windowToSplitName, ImGuiWindow windowToAdd, int imGuiDir, double percent)
   {
      addInstruction(new ImGuiDockingSetupInstruction(windowToSplitName, windowToAdd.getWindowName(), imGuiDir, percent), windowToAdd);
   }

   private void addInstruction(ImGuiDockingSetupInstruction instruction, ImGuiWindow window)
   {
      instructions.add(instruction);
      windows.add(window);
   }

   public void build(int centralDockspaceId)
   {
      for (int i = 0; i < instructions.size(); i++)
      {
         ImGuiDockingSetupInstruction instruction = instructions.get(i);
         if (i == 0)
         {
            String windowName = instruction.getWindowName();
            ImGui.dockBuilderDockWindow(windowName, centralDockspaceId);
            windowsSpaceIds.put(windowName, centralDockspaceId);
         }
         else
         {
            String windowToSplit = instruction.getWindowToSplit();
            int imGuiDir = instruction.getImGuiDir();
            double percent = instruction.getPercent();
            String windowNameToAdd = instruction.getWindowNameToAdd();
            int dockspaceToSplit = windowsSpaceIds.get(windowToSplit);
            ImInt newIdForExistingWindow = new ImInt();
            int dockspaceIdForNewWindow = ImGui.dockBuilderSplitNode(dockspaceToSplit, imGuiDir, (float) percent, null, newIdForExistingWindow);
            windowsSpaceIds.put(windowToSplit, newIdForExistingWindow.get());
            windowsSpaceIds.put(windowNameToAdd, dockspaceIdForNewWindow);
         }
      }

      for (String windowName : windowsSpaceIds.keySet())
      {
         int dockspaceId = windowsSpaceIds.get(windowName);
         ImGui.dockBuilderDockWindow(windowName, dockspaceId);
      }

      ImGui.dockBuilderFinish(centralDockspaceId);
   }

   public ArrayList<ImGuiWindow> getWindows()
   {
      return windows;
   }
}
