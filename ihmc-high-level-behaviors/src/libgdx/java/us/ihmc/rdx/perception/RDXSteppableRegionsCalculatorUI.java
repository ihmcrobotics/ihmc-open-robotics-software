package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegionsCalculationModule;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegionsCalculator;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class RDXSteppableRegionsCalculatorUI
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private final SteppableRegionsCalculationModule steppableRegionsCalculationModule = new SteppableRegionsCalculationModule();
   private Supplier<HeightMapData> heightMapDataSupplier;
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());

   private final ImBoolean drawPatches = new ImBoolean(true);
   private ImGuiPlot numberOfSteppableRegionsPlot;
   private ImGuiPlot wholeAlgorithmDurationPlot;

   private final Stopwatch wholeAlgorithmDurationStopwatch = new Stopwatch();
   private final YoDouble wholeAlgorithmDuration = new YoDouble("wholeAlgorithmDuration", yoRegistry);

   private ImGuiPanel imguiPanel;
   private final List<RDXCVImagePanel> steppabilityPanels = new ArrayList<>();
   private final List<RDXCVImagePanel> steppableRegionsPanels = new ArrayList<>();

   private ImGuiStoredPropertySetTuner parameterTuner;
   private RDXPlanarRegionsGraphic planarRegionsGraphic;
   private int cellsPerSide;
   private final SteppableRegionCalculatorParameters parameters = new SteppableRegionCalculatorParameters();

   public void create(Supplier<HeightMapData> heightMapDataSupplier)
   {
      this.heightMapDataSupplier = heightMapDataSupplier;

      imguiPanel = new ImGuiPanel("Steppable Region Extraction", this::renderImGuiWidgets);
      parameterTuner = new ImGuiStoredPropertySetTuner("Steppable Region Parameters");
      parameterTuner.create(parameters);

      cellsPerSide = heightMapDataSupplier.get().getCellsPerAxis();
      for (int i = 0; i < SteppableRegionsCalculationModule.yawDiscretizations; i++)
      {
         RDXCVImagePanel steppabilityPanel = new RDXCVImagePanel("Raw Steppability " + i, cellsPerSide, cellsPerSide);
         RDXCVImagePanel panel = new RDXCVImagePanel("Steppable Regions " + i, cellsPerSide, cellsPerSide);
         steppabilityPanels.add(steppabilityPanel);
         steppableRegionsPanels.add(panel);
         imguiPanel.addChild(steppabilityPanel.getVideoPanel());
         imguiPanel.addChild(panel.getVideoPanel());
      }

      numberOfSteppableRegionsPlot = new ImGuiPlot(labels.get("Number of steppable regions"), 1000, 300, 50);
      wholeAlgorithmDurationPlot = new ImGuiPlot(labels.get("Whole algorithm duration"), 1000, 300, 50);

      planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   }

   volatile boolean processing = false;
   volatile boolean needToDraw = false;

   public void extractSteppableRegions()
   {
      extractSteppableRegions(null);
   }

   public void extractSteppableRegions(Runnable runWhenFinished)
   {
      if (enabled.get())
      {
         if (!processing)
         {
            HeightMapData heightMapData = heightMapDataSupplier.get();
            resize(heightMapData.getCellsPerAxis());
            processing = true;
            wholeAlgorithmDurationStopwatch.start();
            ThreadTools.startAsDaemon(() -> processingThread(runWhenFinished, heightMapData), getClass().getSimpleName() + "Processing");
         }

         if (needToDraw)
         {
            needToDraw = false;

            render2DPanels();
            renderSteppableRegions();

            processing = false;
         }
      }
   }

   private void processingThread(Runnable runWhenFinished, HeightMapData heightMapData)
   {
      steppableRegionsCalculationModule.setSteppableRegionsCalculatorParameters(parameters);
      steppableRegionsCalculationModule.compute(heightMapData);
      drawRegions();

      wholeAlgorithmDurationStopwatch.suspend();
      wholeAlgorithmDuration.set(wholeAlgorithmDurationStopwatch.lapElapsed());

      if (runWhenFinished != null)
         runWhenFinished.run();

      needToDraw = true;
   }

   private void resize(int newSize)
   {
      if (newSize != cellsPerSide)
      {
         cellsPerSide = newSize;
         OpenCLManager openCLManager = steppableRegionsCalculationModule.getOpenCLManager();
         for (int i = 0; i < SteppableRegionsCalculationModule.yawDiscretizations; i++)
         {
            steppableRegionsPanels.get(i).resize(newSize, newSize, openCLManager);
            steppabilityPanels.get(i).resize(newSize, newSize, openCLManager);
         }
      }
   }

   private void drawRegions()
   {
      for (int i = 0; i < SteppableRegionsCalculationModule.yawDiscretizations; i++)
      {
         RDXCVImagePanel panel = steppableRegionsPanels.get(i);
         SteppableRegionsCalculator.SteppableRegionsEnvironmentModel environmentModel = steppableRegionsCalculationModule.getRegionEnvironments().get(i);
         if (panel.getVideoPanel().getIsShowing().get() && drawPatches.get())
         {
            int size = panel.getBytedecoImage().getImageHeight();
            for (SteppableRegionsCalculator.SteppableRegionDataHolder region : environmentModel.getRegions())
            {
               for (SteppableRegionsCalculator.SteppableCell cell : region.getCells())
               {
                  int x = cell.getX();
                  int y = cell.getY();

                  int row =  size - x - 1;
                  int column = size - y - 1;

                  int r = (region.regionNumber + 1) * 312 % 255;
                  int g = (region.regionNumber + 1) * 123 % 255;
                  int b = (region.regionNumber + 1) * 231 % 255;
                  BytePointer pixel = panel.getBytedecoImage().getBytedecoOpenCVMat().ptr(row, column);
                  pixel.put(0, (byte) r);
                  pixel.put(1, (byte) g);
                  pixel.put(2, (byte) b);
               }
            }
         }

         panel = steppabilityPanels.get(i);
         if (panel.getVideoPanel().getIsShowing().get() && drawPatches.get())
         {
            BytedecoImage image = steppableRegionsCalculationModule.getSteppableImage().get(i);

            for (int x = 0; x < image.getImageWidth(); x++)
            {
               for (int y = 0; y < image.getImageHeight(); y++)
               {
                  Color color;
                  int status = image.getInt(x, y);
                  if (status == 0)
                     color = Color.WHITE; // valid
                  else if (status == 1)
                     color = Color.BLACK; // cliff top
                  else
                     color = Color.GRAY; // cliff bottom
                  BytePointer pixel = panel.getBytedecoImage().getBytedecoOpenCVMat().ptr(x, y);
                  pixel.put(0, (byte) (color.r * 255));
                  pixel.put(1, (byte) (color.g * 255));
                  pixel.put(2, (byte) (color.b * 255));
               }
            }
         }
      }
   }

   private void render2DPanels()
   {
      steppableRegionsPanels.forEach(RDXCVImagePanel::draw);
      steppabilityPanels.forEach(RDXCVImagePanel::draw);
   }

   /** FIXME: This method filled with allocations. */
   private void renderSteppableRegions()
   {
//      if (!render3DPlanarRegions.get())
//         return;

//      planarRegionsGraphic.generateMeshes(gpuPlanarRegionExtraction.getPlanarRegionsList());
//      planarRegionsGraphic.update();
   }


   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Draw patches"), drawPatches);
      parameterTuner.renderImGuiWidgets();

      ImGui.text("Input height map dimensions: " + cellsPerSide + " x " + cellsPerSide);
      ImGui.checkbox(labels.get("Enabled"), enabled);
      wholeAlgorithmDurationPlot.render(wholeAlgorithmDurationStopwatch.totalElapsed());
//      numberOfSteppableRegionsPlot.render((float) steppableRegionsCalculationModule.getSteppableRegions().get(0).size());
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      // TODO
//      if (render3DPlanarRegions.get())
//         planarRegionsGraphic.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      steppableRegionsCalculationModule.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return imguiPanel;
   }

   public ImBoolean getEnabled()
   {
      return enabled;
   }
}
