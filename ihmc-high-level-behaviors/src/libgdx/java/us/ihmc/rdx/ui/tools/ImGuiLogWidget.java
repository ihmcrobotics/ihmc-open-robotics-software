package us.ihmc.rdx.ui.tools;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.Level;

import java.time.LocalDateTime;
import java.util.LinkedList;

public class ImGuiLogWidget
{
   private final LinkedList<Pair<Integer, String>> logEntries = new LinkedList<>();
   private final String name;

   public ImGuiLogWidget(String name)
   {
      this.name = name;
      logEntries.addLast(Pair.of(Level.INFO.intLevel(), "Log started at " + LocalDateTime.now()));
   }

   public void submitEntry(Pair<Integer, String> entry)
   {
      synchronized (logEntries)
      {
         logEntries.addLast(entry);
      }
   }

   public void submitEntry(Level level, String message)
   {
      submitEntry(level.intLevel(), message);
   }

   public void submitEntry(int intLevel, String message)
   {
      synchronized (logEntries)
      {
         logEntries.addLast(Pair.of(intLevel, message));
      }
   }

   public void renderImGuiWidgets()
   {
      synchronized (logEntries)
      {
         ImGui.text(name + " log : ");
         ImGui.pushItemWidth(ImGui.getWindowWidth() - 10);
         int numLogEntriesToShow = 20;
         while (logEntries.size() > numLogEntriesToShow)
            logEntries.removeFirst();
         for (Pair<Integer, String> logEntry : logEntries)
         {
            Color color = Color.WHITE;
            float[] hsv = new float[3];
            switch (logEntry.getLeft())
            {
               case 100:
               case 200:
                  color = Color.RED;
                  break;
               case 300:
                  Color.YELLOW.toHsv(hsv);
                  color = color.fromHsv(hsv[0], hsv[1], 0.7f);
                  break;
               case 400:
                  color = Color.BLACK;
                  break;
               case 500:
                  color = Color.CYAN;
                  break;
               case 600:
                  color = Color.GREEN;
                  break;
            }
            ImGui.textColored(color.r, color.g, color.b, 1.0f, logEntry.getRight());
         }
      }
      ImGui.popItemWidth();
   }
}
