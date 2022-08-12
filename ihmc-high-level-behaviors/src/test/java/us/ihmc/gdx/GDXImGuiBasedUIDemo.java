package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.apache.logging.log4j.Level;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import rosgraph_msgs.Log;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.tools.ImGuiLogWidget;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.string.StringTools;

import java.lang.reflect.Array;
import java.nio.file.Files;
import java.time.LocalDateTime;
import java.util.ArrayList;

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
   private ImBoolean option = new ImBoolean();
   private Texture iconTexture;
   private int pressCount = 0;

   private ArrayList<Texture> iconTextures = new ArrayList<>();
   private final WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                        "ihmc-high-level-behaviors/src/test/resources");
   private final String pathToFiles[] = new String[] {"leftFoot_depress.png", "flyingCar.png", "hoverboard.png" };

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

//            WorkspaceFile testImageFile = new WorkspaceFile(new WorkspaceDirectory("ihmc-open-robotics-software",
//                                                                                   "ihmc-high-level-behaviors/src/test/resources"),
//                                                            "leftFoot_depress.png");
//            Mat readImage = opencv_imgcodecs.imread(testImageFile.getFilePath().toString());
//            Pixmap pixmap = new Pixmap(readImage.cols(), readImage.rows(), Pixmap.Format.RGBA8888);
//            BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
//            Mat rgba8Mat = new Mat(readImage.rows(), readImage.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
//            opencv_imgproc.cvtColor(readImage, rgba8Mat, opencv_imgproc.COLOR_RGB2RGBA);
//            iconTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));


            for (String pathToFile : pathToFiles)
            {
               WorkspaceFile imageFile = new WorkspaceFile(directory,pathToFile);
               Mat readImage = opencv_imgcodecs.imread(imageFile.getFilePath().toString());
               Pixmap pixmap = new Pixmap(readImage.cols(), readImage.rows(), Pixmap.Format.RGBA8888);
               BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
               Mat rgba8Mat = new Mat(readImage.rows(), readImage.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
               opencv_imgproc.cvtColor(readImage, rgba8Mat, opencv_imgproc.COLOR_RGB2RGBA);
               iconTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
               iconTextures.add(iconTexture);
            }

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



      for (Texture iconTexture : iconTextures)
      {
         if (ImGui.imageButton(iconTexture.getTextureObjectHandle(), 40.0f, 40.0f))
         {
            pressCount++;
         }
         ImGui.text("Press count: " + pressCount);
      }

      // testing imguiImage (not button)
      ImGui.text("testing imguiImage (not button)");
      int testTextureID = iconTextures.get(1).getTextureObjectHandle();
      ImGui.image(testTextureID,40f,40f);



//      if (ImGui.imageButton(iconTexture.getTextureObjectHandle(), 20.0f, 20.0f))
//      {
//         pressCount++;
//      }
//      ImGui.text("Press count: " + pressCount);
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
