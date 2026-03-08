package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.ImGuiViewport;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiStyleVar;
import imgui.flag.ImGuiWindowFlags;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImFloatWrapper.FloatSupplier;

import java.util.ArrayList;
import java.util.function.Function;

public class RDXStatusBar
{
   public record Element(Runnable renderContent, FloatSupplier heightNeeded, Function<Boolean, Boolean> renderIcon, ImBoolean selected) { }
   private final ArrayList<Element> elements = new ArrayList<>();
   private float currentHeight = -1.0f;
   private float desiredHeight = -1.0f;
   private long lastTime = -1;

   public void render()
   {
      if (currentHeight < 0.0f)
      {
         currentHeight = ImGui.getFrameHeight();
         desiredHeight = currentHeight;
      }

      Element selected = null;
      for (int i = 0; i < elements.size(); i++)
         if (elements.get(i).selected.get())
         {
            selected = elements.get(i);
            break;
         }

      ImGuiViewport viewport = ImGui.getMainViewport();
      int size = ImGui.getFontSize();
      desiredHeight = ImGui.getFrameHeight() + (selected == null ? 0.0f : selected.heightNeeded.getAsFloat());
      long now = System.nanoTime();
      if (lastTime > 0)
      {
         float dt = (now - lastTime) * 0.000000001f;
         float error = Math.abs(desiredHeight - currentHeight);
         currentHeight += Math.signum(desiredHeight - currentHeight) * Math.min(error, dt * size * 100.0f);
      }
      lastTime = now;

      ImGui.setNextWindowPos(viewport.getWorkPosX(),
                             viewport.getWorkPosY() + viewport.getWorkSizeY() - currentHeight);
      ImGui.setNextWindowSize(viewport.getWorkSizeX(), currentHeight);
      ImGui.setNextWindowViewport(viewport.getID());
      ImGui.pushStyleVar(ImGuiStyleVar.WindowMinSize, 0.0f, 0.0f);
      ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, size * 0.7f, size * 0.2f);
      ImGui.pushStyleColor(ImGuiCol.WindowBg, ImGui.getColorU32(ImGuiCol.MenuBarBg));
      int flags = ImGuiWindowFlags.NoDecoration;
      flags |= ImGuiWindowFlags.NoDocking;
      flags |= ImGuiWindowFlags.AlwaysAutoResize;
      flags |= ImGuiWindowFlags.NoSavedSettings;
      flags |= ImGuiWindowFlags.NoFocusOnAppearing;
      flags |= ImGuiWindowFlags.NoNav;
      ImGui.begin("##Status Bar", flags);

      if (selected != null)
         selected.renderContent.run();

      ImGui.setCursorPosY(ImGui.getWindowHeight() - ImGui.getFrameHeight() + size * 0.2f);
      ImGui.text("Ready");
      ImGui.sameLine();

      for (int i = 0; i < elements.size(); i++)
      {
         ImGui.setCursorPosX(ImGui.getWindowWidth() - (ImGui.getFrameHeight() + ImGui.getStyle().getItemSpacingX()) * (elements.size() - i));
         if (elements.get(i).renderIcon.apply(elements.get(i).selected.get()))
            for (int j = 0; j < elements.size(); j++)
               elements.get(j).selected.set(!elements.get(i).selected.get() && j == i);
         ImGui.sameLine();
      }

      ImGui.end();
      ImGui.popStyleVar(2);
      ImGui.popStyleColor(1);
   }

   public Element add(Runnable renderContent, FloatSupplier heightNeeded, Function<Boolean, Boolean> renderIcon)
   {
      Element element = new Element(renderContent, heightNeeded, renderIcon, new ImBoolean(false));
      elements.add(element);
      return element;
   }

   public void remove(Element element)
   {
      elements.remove(element);
   }
}
