package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.avatar.logging.PlanarRegionsReplayBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.*;

public class RDXLoggingDevelopmentUI
{

   private final String WINDOW_NAME = "Logging Development UI";
   private final int MAX_TICK_LENGTH = 1000000; //Buffer/log length
   private final int TICK_PERIOD = 200; //Measured in ms

   private RDXBaseUI baseUI;

   private RDXPlanarRegionsGraphic planarRegionsGraphic;

   private Random random;

   private Stopwatch stopwatch;

   private IntraprocessYoVariableLogger yoLogger;
   private YoBuffer yoBuffer;

   private PlanarRegionsReplayBuffer planarRegionsReplayBuffer;
   private PlanarRegionsListLogger planarRegionsListLogger;

   private PlanarRegionsList regions;
   private YoLong planarRegionsID;

   private YoRegistry registry;

   private Timer timer;

   private ImInt t = new ImInt();
   private boolean updateWithTick = true;

   private boolean positionAdjusted = false;
   private int timeBoundsLeft = 0;
   private int timeBoundsRight = 0;
   private float zoomMultiplier = 1;

   public RDXLoggingDevelopmentUI()
   {
      LogTools.info("Starting UI");
      baseUI = new RDXBaseUI(WINDOW_NAME);

      random = new Random();

      registry = new YoRegistry("registry");

      yoLogger = new IntraprocessYoVariableLogger(getClass().getSimpleName(), registry, MAX_TICK_LENGTH, 1);
      planarRegionsListLogger = new PlanarRegionsListLogger(getClass().getSimpleName(), MAX_TICK_LENGTH);

      planarRegionsID = new YoLong("planarRegionsID", registry);

      yoBuffer = new YoBuffer(MAX_TICK_LENGTH);
      yoBuffer.addVariable(planarRegionsID);

      planarRegionsReplayBuffer = new PlanarRegionsReplayBuffer(MAX_TICK_LENGTH);

      timer = new Timer();
      stopwatch = new Stopwatch();
   }

   public void launch()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            planarRegionsGraphic = new RDXPlanarRegionsGraphic();
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionsGraphic);

            //Task gets state
            timer.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  regions = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(random, 5, 10, 10, 3);
               }
            }, 0, TICK_PERIOD);

            stopwatch.start();
            yoLogger.start();
            planarRegionsListLogger.start();

            timer.scheduleAtFixedRate(new TimerTask()
            {
               @Override
               public void run()
               {
                  yoBuffer.tickAndWriteIntoBuffer();
                  yoLogger.update((long) (stopwatch.totalElapsed()));

                  if (updateWithTick)
                     t.set(yoBuffer.getCurrentIndex() - 1);


                  long time = System.currentTimeMillis();
                  planarRegionsReplayBuffer.putAndTick(time, regions);
                  planarRegionsListLogger.update(time, regions);
               }
            }, 50, TICK_PERIOD);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            PlanarRegionsList list = (PlanarRegionsList) planarRegionsReplayBuffer.get(t.get());
            if (list != null)
               planarRegionsGraphic.generateMeshesAsync(list);
            planarRegionsGraphic.update();

            //Random Number Monitor
            ImGui.begin("Time Control");
            ImGui.text((int) (stopwatch.totalElapsed()) + " s");
            ImGui.text("Current time: " + System.currentTimeMillis());

            if (ImGui.sliderInt("Time", t.getData(), 0, yoBuffer.getCurrentIndex() - 1, ""))
            {
               positionAdjusted = true;
               updateWithTick = false;
            }

            ImGui.sameLine();
            if (ImGui.inputInt("tBox", t))
            {
               positionAdjusted = true;
               updateWithTick = false;
            }

            if (positionAdjusted)
            {
               if (updateWithTick)
               {
                  timeBoundsLeft = 0;
                  timeBoundsRight = 0;
               }
               else
               {
                  timeBoundsLeft = t.get() - Math.round(100 * zoomMultiplier);
                  timeBoundsRight = t.get() + Math.round(100 * zoomMultiplier);

                  if (timeBoundsLeft < 0)
                     timeBoundsLeft = 0;
                  if (timeBoundsRight > yoBuffer.getCurrentIndex() - 1)
                     timeBoundsRight = yoBuffer.getCurrentIndex() - 1;
               }

               positionAdjusted = false;
            }

            if (updateWithTick)
               ImGui.sliderInt("Zoom: ", new int[] {0}, timeBoundsLeft, timeBoundsRight, "");
            else
               ImGui.sliderInt("Zoom: ", t.getData(), timeBoundsLeft, timeBoundsRight);

            //Lots of buttons
            ImGui.sameLine();
            if (ImGui.button("In"))
            {
               positionAdjusted = true;
               zoomMultiplier /= 2;
            }
            ImGui.sameLine();
            if (ImGui.button("Out"))
            {
               positionAdjusted = true;
               zoomMultiplier *= 2;
            }

            if (ImGui.button("<<"))
            {
               if (updateWithTick)
                  positionAdjusted = true;
               updateWithTick = false;
               t.set(Math.max(t.get() - 50, timeBoundsLeft));
            }
            ImGui.sameLine();
            if (ImGui.button("<"))
            {
               if (updateWithTick)
                  positionAdjusted = true;
               updateWithTick = false;
               t.set(Math.max(t.get() - 1, timeBoundsLeft));
            }
            ImGui.sameLine();
            if (ImGui.button(">"))
            {
               if (updateWithTick)
                  positionAdjusted = true;
               updateWithTick = false;
               t.set(Math.min(t.get() + 1, timeBoundsRight));
            }
            ImGui.sameLine();
            if (ImGui.button(">>"))
            {
               if (updateWithTick)
                  positionAdjusted = true;
               updateWithTick = false;
               t.set(Math.min(t.get() + 50, timeBoundsRight));
            }
            ImGui.sameLine();
            if (ImGui.button("Go Live"))
            {
               positionAdjusted = true;
               updateWithTick = true;
            }

            ImGui.text("Buffer Utilization: " + (int) (100 * (yoBuffer.getCurrentIndex() - 1) / yoBuffer.getBufferSize()) + "%");

            ImGui.end();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      RDXLoggingDevelopmentUI ui = new RDXLoggingDevelopmentUI();
      ui.launch();
   }
}
