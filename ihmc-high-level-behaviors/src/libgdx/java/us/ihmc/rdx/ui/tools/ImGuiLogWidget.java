package us.ihmc.rdx.ui.tools;

import com.badlogic.gdx.graphics.Color;
import imgui.flag.ImGuiCol;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.logging.log4j.Level;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.time.Instant;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.LinkedList;

public class ImGuiLogWidget
{
   private static final int WARN_COLOR;
   static
   {
      float[] hsv = new float[3];
      Color.YELLOW.toHsv(hsv);
      WARN_COLOR = new Color().fromHsv(hsv[0], hsv[1], 0.7f).toIntBits();
   }

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private int entryLimit = 0;
   private float childHeight = 200.0f;
   private final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("H:mm:ss:SSS").withZone(ZoneId.systemDefault());
   private record Entry(Instant instant, Level level, String message) { };
   private final LinkedList<Entry> logEntries = new LinkedList<>();
   private final ImBoolean scrollToEnd = new ImBoolean(true);
   private final Notification wouldScrollToEnd = new Notification();

   public ImGuiLogWidget()
   {
      submitEntry(Level.INFO, "Log started.");
   }

   public synchronized void submitEntry(Level level, String message)
   {
      submitEntry(Instant.now(), level, message);
   }

   public synchronized void submitEntry(Instant instant, Level level, String message)
   {
      logEntries.addLast(new Entry(instant, level, message));
      wouldScrollToEnd.set();
   }

   public synchronized void renderImGuiWidgets()
   {
      ImGui.pushStyleColor(ImGuiCol.ChildBg, ImGui.getColorU32(ImGuiCol.FrameBg));
      ImGui.beginChild(labels.getHidden("logChild"), 0.0f, childHeight);
      ImGui.pushFont(ImGuiTools.getConsoleFont());

      if (entryLimit > 0)
         while (logEntries.size() > entryLimit)
            logEntries.removeFirst();

      for (Entry logEntry : logEntries)
      {
         Level level = logEntry.level();

         int color;
         if (level.equals(Level.FATAL) || level.equals(Level.ERROR))
         {
            color = ImGuiTools.RED;
         }
         else if (level.equals(Level.WARN))
         {
            color = WARN_COLOR;
         }
         else if (level.equals(Level.DEBUG))
         {
            color = ImGuiTools.CYAN;
         }
         else if (level.equals(Level.TRACE))
         {
            color = ImGuiTools.GREEN;
         }
         else
         {
            color = ImGui.getColorU32(ImGuiCol.Text); // INFO color
         }

         String textToDisplay = "%s %s".formatted(dateTimeFormatter.format(logEntry.instant()),
                                                       logEntry.message());
         ImGui.textColored(color, textToDisplay);
      }

      if (wouldScrollToEnd.poll() && scrollToEnd.get())
         ImGui.setScrollHereY();

      ImGui.popFont();
      ImGui.endChild();
      ImGui.popStyleColor();

      if (ImGui.checkbox(labels.get("Scroll to End"), scrollToEnd))
         if (scrollToEnd.get())
            wouldScrollToEnd.set();
   }

   public void setScrollableAreaHeight(float childHeight)
   {
      this.childHeight = childHeight;
   }

   /** 0 for unlimited. */
   public void setEntryLimit(int entryLimit)
   {
      this.entryLimit = entryLimit;
   }
}
