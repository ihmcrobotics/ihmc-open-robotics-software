package us.ihmc.communication.ros2.sync;

import imgui.ImGui;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.pubsub.common.Guid;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

import java.time.Instant;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;

public class RDXROS2PeerClockTest
{
   public RDXROS2PeerClockTest()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         ROS2PeerClockOffsetEstimator clockEstimator;
         final DateTimeFormatter formatter = DateTimeFormatter.ofPattern("HH:mm:ss:SSS");
         final Map<Guid, float[]> plots = new HashMap<>();
         final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
         double history = 5.0;
         int samples = 1000;
         final Throttler plotThottler = new Throttler().setPeriod(history / samples);

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getImGuiPanelManager().addPanel("Main", this::renderImGuiWidgets);

            ROS2Node ros2Node = new ROS2NodeBuilder().build("peer_clock_test");
            clockEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
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

         private void renderImGuiWidgets()
         {
            Instant now = Instant.now();

            ImGui.text("Current time:");
            ImGui.pushFont(ImGuiTools.getBigFont());
            ImGui.text(formatter.format(now.atZone(ZoneId.systemDefault())));
            ImGui.popFont();

            boolean shiftPlots = plotThottler.run();

            for (ROS2PeerClockOffsetEstimatorPeer peer : clockEstimator.getPeerList())
            {
               if (peer.isAlive(now))
               {
                  ImGui.separator();
                  ImGui.text("Peer %s time (peer frame):".formatted(peer.getGuid()));
                  ImGui.pushFont(ImGuiTools.getBigFont());
                  ImGui.text(formatter.format(peer.getPeerTimeInPeerFrame(now).atZone(ZoneId.systemDefault())));
                  ImGui.popFont();
                  long offsetInMillis =  peer.getPeerClockOffset().toMillis();
                  ImGui.text("Offset (ms):");

                  float[] plot = plots.get(peer.getGuid());
                  if (plot == null)
                  {
                     plot = new float[samples];
                     plots.put(peer.getGuid(), plot);
                  }

                  if (shiftPlots)
                     for (int i = 0; i < plot.length - 1; i++)
                        plot[i] = plot[i + 1];

                  plot[plot.length - 1] = offsetInMillis;

                  ImGui.pushFont(ImGuiTools.getBigFont());
                  ImGui.plotLines(labels.get("Offset (ms)", peer.getGuid().hashCode()),
                                  plot,
                                  samples,
                                  0,
                                  "%d ms".formatted(offsetInMillis),
                                  -30.0f,
                                  30.0f,
                                  ImGui.getColumnWidth(),
                                  70.0f);
                  ImGui.popFont();
               }
            }
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXROS2PeerClockTest();
   }
}
