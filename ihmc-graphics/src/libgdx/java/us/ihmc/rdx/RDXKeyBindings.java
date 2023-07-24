package us.ihmc.rdx;

import imgui.flag.ImGuiCond;
import imgui.flag.ImGuiKey;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;

import javax.annotation.Nullable;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

public class RDXKeyBindings
{
   private static final int WINDOW_WIDTH = 600;
   private static final int WINDOW_HEIGHT = 400;

   @Nullable
   private KeyBindingsSection currentSection;
   private final Set<KeyBindingsSection> sections = new HashSet<>();

   private static class KeyBindingsSection
   {
      private final String description;
      // Function to key map
      private final TreeMap<String, String> registry = new TreeMap<>();

      public KeyBindingsSection(String description)
      {
         this.description = description;
      }

      public void renderSection()
      {
         for (Map.Entry<String, String> entry : registry.entrySet())
         {
            String function = entry.getKey();
            String key = entry.getValue();
            ImGui.tableNextRow();
            ImGui.tableSetColumnIndex(0);
            ImGui.alignTextToFramePadding();
            ImGui.text(function);
            ImGui.tableSetColumnIndex(1);
            ImGui.text(key);
         }
      }
   }

   public void nextSection(String sectionName)
   {
      if (currentSection != null)
      {
         for (KeyBindingsSection section : sections)
         {
            if (section.description.equals(sectionName))
               return;
         }

         sections.add(currentSection);
      }
      currentSection = new KeyBindingsSection(sectionName);
   }

   public void register(String function, String key) throws IllegalArgumentException
   {
      if (currentSection == null)
      {
         throw new IllegalStateException("You must call RDXKeyBindings#nextSection() first before registering a key bind.");
      }
      currentSection.registry.put(function, key);
   }

   public void renderKeyBindingsTable()
   {
      if (ImGui.isKeyDown(ImGui.getKeyIndex(ImGuiKey.Tab)))
      {
         ImGui.setNextWindowViewport(ImGui.getMainViewport().getID());
         float x = ImGui.getMainViewport().getCenterX() - (WINDOW_WIDTH * 0.5f);
         float y = ImGui.getMainViewport().getCenterY() - (WINDOW_HEIGHT * 0.5f);
         ImGui.setNextWindowPos(x, y, ImGuiCond.Always);
         ImGui.begin("##keyBindingsWindow", ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize);
         ImGui.beginChild("##keyBindingsWindowChild", WINDOW_WIDTH, WINDOW_HEIGHT);

         for (KeyBindingsSection section : sections)
         {
            if (ImGui.beginTable("##keyBindingsTable_" + section.description, 2))
            {
               ImGui.tableSetupColumn(section.description);
               ImGui.tableSetupColumn("Key");
               ImGui.tableHeadersRow();
               section.renderSection();
               ImGui.endTable();
            }
         }

         ImGui.endChild();
         ImGui.end();
      }
   }
}
