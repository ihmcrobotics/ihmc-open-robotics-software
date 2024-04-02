package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.logging.log4j.Level;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImGuiLogWidget;
import us.ihmc.rdx.ui.widgets.ImGuiHandWidget;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.string.StringTools;

import java.time.LocalDateTime;

public class RDXImGuiBasedUIDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Demo");
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final ImGuiMovingPlot renderPlot = new ImGuiMovingPlot("render count", 1000, 300, 30);
   private final ImGuiLogWidget logWidget = new ImGuiLogWidget();
   private long renderCount = 0;
   private final ImBoolean option = new ImBoolean();
   private int pressCount = 0;
   private final ImString textForArea = new ImString();
   private RDX3DPanelToolbarButton flyingCarButton;
   private final ImGuiHandWidget handWidget = new ImGuiHandWidget();

   public RDXImGuiBasedUIDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            baseUI.getImGuiPanelManager().addPanel("Window 1", RDXImGuiBasedUIDemo.this::renderWindow1);
            baseUI.getImGuiPanelManager().addPanel("Window 2", RDXImGuiBasedUIDemo.this::renderWindow2);
            baseUI.getImGuiPanelManager().addPanel("Window 3", RDXImGuiBasedUIDemo.this::renderWindow3);

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

            flyingCarButton = baseUI.getPrimary3DPanel().addToolbarButton();
            flyingCarButton.loadAndSetIcon("icons/flyingCar.png");
            flyingCarButton.setOnPressed(() -> pressCount++);
            flyingCarButton.setTooltipText("Tooltip text");

            RDX3DPanelToolbarButton hoverboardButton = baseUI.getPrimary3DPanel().addToolbarButton();
            hoverboardButton.loadAndSetIcon("icons/hoverboard.png");

            RDX3DPanel second3DPanel = new RDX3DPanel("Second 3D View", true);
            baseUI.add3DPanel(second3DPanel);

            logWidget.setScrollableAreaHeight(100.0f);
            logWidget.submitEntry(Level.WARN, "Test warning level.");
            logWidget.submitEntry(Level.ERROR, "Test error level.");
            logWidget.submitEntry(Level.DEBUG, "Test debug level.");
            logWidget.submitEntry(Level.FATAL, "Test fatal level.");
            logWidget.submitEntry(Level.TRACE, "Test trace level.");

            textForArea.set("Text for area");
         }

         @Override
         public void render()
         {
            // call update() methods here
            if (flyingCarButton.getDown())
               flyingCarButton.setTooltipText("Mouse down.");
            else if (flyingCarButton.getHovered())
               flyingCarButton.setTooltipText("Mouse hovered.");

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
      if (ImGui.button("I'm a Button!"))
         logWidget.submitEntry(Level.INFO, "Button pressed!");

      float[] values = new float[100];
      for (int i = 0; i < 100; i++)
      {
         values[i] = i;
      }
      ImGui.plotLines("Histogram", values, 100);
      renderPlot.calculate(renderCount++);

      logWidget.renderImGuiWidgets();

      ImGui.text("Toolbar button press count: " + pressCount);
      ImGuiTools.inputText("Text area", textForArea);

      handWidget.render(RobotSide.LEFT, ImGui.getFontSize(), false);
   }

   private void renderWindow2()
   {
      ImGui.text("I'm in Window 2!");
   }

   private void renderWindow3()
   {
      ImGui.text("I'm in Window 3!");
   }

   public static void main(String[] args)
   {
      new RDXImGuiBasedUIDemo();
   }
}
