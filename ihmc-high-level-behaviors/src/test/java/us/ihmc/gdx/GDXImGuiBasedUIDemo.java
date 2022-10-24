package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.logging.log4j.Level;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDX3DPanelToolbarButton;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.tools.ImGuiLogWidget;
import us.ihmc.tools.string.StringTools;

import java.time.LocalDateTime;

public class GDXImGuiBasedUIDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "Demo");
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot renderPlot = new ImGuiMovingPlot("render count", 1000, 300, 30);
   private final ImGuiLogWidget logWidget = new ImGuiLogWidget("Log");
   private long renderCount = 0;
   private final ImBoolean option = new ImBoolean();
   private int pressCount = 0;

   public GDXImGuiBasedUIDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            baseUI.getImGuiPanelManager().addPanel("Window 1", GDXImGuiBasedUIDemo.this::renderWindow1);
            baseUI.getImGuiPanelManager().addPanel("Window 2", GDXImGuiBasedUIDemo.this::renderWindow2);
            baseUI.getImGuiPanelManager().addPanel("Window 3", GDXImGuiBasedUIDemo.this::renderWindow3);

            baseUI.getPrimary3DPanel().addImGuiOverlayAddition(() ->
            {
               if (ImGui.isWindowHovered() && ImGui.isMouseClicked(ImGuiMouseButton.Right))
               {
                  ImGui.openPopup("Popup");
               }
               if (ImGui.beginPopup("Popup"))
               {
                  ImGui.menuItem("Button");
                  ImGui.menuItem("Option", null, option);
                  if (ImGui.menuItem("Cancel"))
                     ImGui.closeCurrentPopup();
                  ImGui.endPopup();
               }
            });

            GDX3DPanelToolbarButton flyingCarButton = baseUI.getPrimary3DPanel().addToolbarButton();
            flyingCarButton.loadAndSetIcon("icons/flyingCar.png");
            flyingCarButton.setOnPressed(() -> pressCount++);
            flyingCarButton.setTooltipText("Tooltip text");

            GDX3DPanelToolbarButton hoverboardButton = baseUI.getPrimary3DPanel().addToolbarButton();
            hoverboardButton.loadAndSetIcon("icons/hoverboard.png");

            GDX3DPanel second3DPanel = new GDX3DPanel("Second 3D View", 2, true);
            baseUI.add3DPanel(second3DPanel);

            logWidget.submitEntry(Level.WARN, "WARN at " + LocalDateTime.now());
            logWidget.submitEntry(Level.ERROR, "ERROR at " + LocalDateTime.now());
            logWidget.submitEntry(Level.DEBUG, "DEBUG at " + LocalDateTime.now());
            logWidget.submitEntry(Level.FATAL, "FATAL at " + LocalDateTime.now());
            logWidget.submitEntry(Level.TRACE, "TRACE at " + LocalDateTime.now());
         }

         @Override
         public void render()
         {
            // call update() methods here

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   private void renderWindow1()
   {
      if (ImGui.beginTabBar("main"))
      {
         if (ImGui.beginTabItem("Window"))
         {
            ImGui.text("Tab bar detected!");
            ImGui.endTabItem();
         }
         ImGui.endTabBar();
      }
      ImGui.text(StringTools.format3D("Time: {} s", stopwatch.totalElapsed()).get());
      ImGui.button("I'm a Button!");
      float[] values = new float[100];
      for (int i = 0; i < 100; i++)
      {
         values[i] = i;
      }
      ImGui.plotLines("Histogram", values, 100);
      renderPlot.calculate(renderCount++);

      logWidget.renderImGuiWidgets();

      ImGui.text("Toolbar button press count: " + pressCount);
   }

   private void renderWindow2()
   {
   }

   private void renderWindow3()
   {
   }

   public static void main(String[] args)
   {
      new GDXImGuiBasedUIDemo();
   }
}
