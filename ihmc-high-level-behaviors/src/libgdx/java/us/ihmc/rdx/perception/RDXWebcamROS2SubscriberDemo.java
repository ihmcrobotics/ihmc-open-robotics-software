package us.ihmc.rdx.perception;

import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.imgui.ImPlotDoublePlot;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Throttler;

/**
 * Subscribes to ROS 2 webcam images with best performance.
 * <p>
 * TODO Okay, well it's got some potential optimizations. See commented code.
 */
public class RDXWebcamROS2SubscriberDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ROS 2 Webcam Subscriber");
   private final RDXPanel diagnosticPanel = new RDXPanel("Diagnostics", this::renderImGuiWidgets);
   //   private ImGuiVideoPanel videoPanel;
   private RDXBytedecoImagePanel cvImagePanel;
   private final ImPlotFrequencyPlot receiveFrequencyPlot = new ImPlotFrequencyPlot("Receive frequency");
   private final ImPlotDoublePlot delayPlot = new ImPlotDoublePlot("Network transmission duration");
   private final ImPlotStopwatchPlot decodeDurationPlot = new ImPlotStopwatchPlot("Decode duration");
   private final ImPlotFrequencyPlot transferFrequencyPlot = new ImPlotFrequencyPlot("Transfer frequency");
   private final ImPlotFrequencyPlot uiUpdateFrequencyPlot = new ImPlotFrequencyPlot("UI update frequency");
   private final ImPlotStopwatchPlot copyBytesDurationPlot = new ImPlotStopwatchPlot("Copy bytes duration");
   private final Throttler transferThrottler = new Throttler();
   private RealtimeROS2Node realtimeROS2Node;
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final SampleInfo sampleInfo = new SampleInfo();
   private boolean readyToReceive = false;
   private volatile boolean gotImageYet = false;
   private final Object syncObject = new Object();
   private int imageWidth = -1;
   private int imageHeight = -1;
   private final byte[] messageDataHeapArray = new byte[25000000];
   private BytePointer messageEncodedBytePointer;
   private Mat inputJPEGMat;
   private Mat inputYUVI420Mat;
   private Mat bgr8Mat;

   public RDXWebcamROS2SubscriberDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videosub");
            ROS2Tools.createCallbackSubscription(realtimeROS2Node, PerceptionAPI.BIG_VIDEO_TEST, subscriber ->
            {
               receiveFrequencyPlot.ping();

               synchronized (syncObject)
               {
                  videoPacket.getData().resetQuick();
                  subscriber.takeNextData(videoPacket, sampleInfo);
                  gotImageYet = true;
               }

               //             videoSwapData.accessOnHighPriorityThread(data ->
               //             {
               //                data.incomingDataMessage(subscriber);
               //             });
            });
            realtimeROS2Node.spin();

            cvImagePanel = new RDXBytedecoImagePanel("Video1", 1920, 1080);
            baseUI.getImGuiPanelManager().addPanel(cvImagePanel.getImagePanel());

            messageEncodedBytePointer = new BytePointer(25000000);
            //                  inputJPEGYUVI420Mat = new Mat(25000000);
            inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
            inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
         }

         @Override
         public void render()
         {
            readyToReceive = true;

            //               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
            //               {
            //                  uiUpdateFrequencyPlot.ping();
            //                  data.updateOnUIThread(swapCVPanel.getVideoPanel());
            //               });

            if (gotImageYet)
            {
               synchronized (syncObject)
               {
                  delayPlot.addValue(TimeTools.calculateDelay(videoPacket.getAcquisitionTimeSecondsSinceEpoch(),
                                                              videoPacket.getAcquisitionTimeAdditionalNanos()));

                  if (videoPacket.getImageWidth() != imageWidth || videoPacket.getImageHeight() != imageHeight)
                  {
                     imageWidth = videoPacket.getImageWidth();
                     imageHeight = videoPacket.getImageHeight();

                     cvImagePanel.resize(imageWidth, imageHeight, null);
                     bgr8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);
                  }

                  IDLSequence.Byte imageEncodedTByteArrayList = videoPacket.getData();
                  imageEncodedTByteArrayList.toArray(messageDataHeapArray);
                  messageEncodedBytePointer.put(messageDataHeapArray, 0, imageEncodedTByteArrayList.size());
                  messageEncodedBytePointer.limit(imageEncodedTByteArrayList.size());

                  inputJPEGMat.cols(imageEncodedTByteArrayList.size());
                  inputJPEGMat.data(messageEncodedBytePointer);
               }

               // imdecode takes the longest by far out of all this stuff
               decodeDurationPlot.start();
               opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);
               decodeDurationPlot.stop();

               opencv_imgproc.cvtColor(inputYUVI420Mat, cvImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_YUV2RGBA_I420);
            }

            cvImagePanel.draw();

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

   private void renderImGuiWidgets()
   {
      ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
      receiveFrequencyPlot.renderImGuiWidgets();
      delayPlot.renderImGuiWidgets();
      decodeDurationPlot.renderImGuiWidgets();
      //         transferFrequencyPlot.renderImGuiWidgets();
      //         copyBytesDurationPlot.renderImGuiWidgets();
      //         uiUpdateFrequencyPlot.renderImGuiWidgets();
   }

   public static void main(String[] args)
   {
      new RDXWebcamROS2SubscriberDemo();
   }
}
