package us.ihmc.gdx.perception;

import controller_msgs.msg.dds.BigVideoPacket;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.ZeroCopySwapReference;

import java.io.IOException;

public class WebcamROS2SubscriberDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "ROS 2 Webcam Subscriber");
   private final ImGuiPanel diagnosticPanel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
   private ImPlotStopwatchPlot acceptMessagePerformancePlot = new ImPlotStopwatchPlot("Process");
   private ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Receive frequency");
   private RealtimeROS2Node realtimeROS2Node;
   private ZeroCopySwapReference<BigVideoSwapData> videoSwapData = new ZeroCopySwapReference<>(BigVideoSwapData::new);
   private boolean readyToReceive = false;
   private int imageWidth = -1;
   private int imageHeight = -1;

   public WebcamROS2SubscriberDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videopub");
            try
            {
               realtimeROS2Node.createSubscription(BigVideoPacket.getPubSubType().get(), subscriber ->
               {
                  videoSwapData.accessOnHighPriorityThread(data ->
                  {
                     data.incomingDataMessage(subscriber);
                  });
               }, "/video_test", ROS2QosProfile.BEST_EFFORT());
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
            realtimeROS2Node.spin();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Video", false);
                  baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getVideoPanel());

                  acceptMessagePerformancePlot = new ImPlotStopwatchPlot("acceptMessage");

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (true)
                     {
                        videoSwapData.accessOnLowPriorityThread(data ->
                        {
                           swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(panelData ->
                           {
                              imageWidth = data.getVideoPacket().getImageWidth();
                              imageHeight = data.getVideoPacket().getImageHeight();

                              panelData.updateOnImageUpdateThread(imageWidth, imageHeight);

                              BytePointer bytePointer = panelData.getRGBA8Mat().ptr(0);
                              for (int i = 0; i < data.getVideoPacket().getData().size(); i++)
                              {
                                 bytePointer.put(i, data.getVideoPacket().getData().get(i));
                              }
                           });
                        });
                     }
                  }, "Transfer");
               }

               readyToReceive = true;

               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
               {
                  data.updateOnUIThread(swapCVPanel.getVideoPanel());
               });

               baseUI.renderBeforeOnScreenUI();
               baseUI.renderEnd();
            }
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      if (nativesLoadedActivator.peek())
      {
         ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
         acceptMessagePerformancePlot.renderImGuiWidgets();
         readFrequencyPlot.renderImGuiWidgets();
      }
   }

   public static void main(String[] args)
   {
      new WebcamROS2SubscriberDemo();
   }
}
