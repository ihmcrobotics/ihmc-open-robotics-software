package us.ihmc.communication.ros2.sync;

import imgui.ImGui;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

import java.time.Instant;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

public class RDXROS2PeerClockTest
{
   public RDXROS2PeerClockTest()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         ROS2PeerClockOffsetEstimator clockEstimator;
         DateTimeFormatter formatter = DateTimeFormatter.ofPattern("HH:mm:ss:SSS");

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

            for (ROS2PeerClockOffsetEstimatorPeer peer : clockEstimator.getPeerList())
            {
               ImGui.text("Peer %s time:".formatted(peer.getGuid()));
            }
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXROS2PeerClockTest();
   }
}
