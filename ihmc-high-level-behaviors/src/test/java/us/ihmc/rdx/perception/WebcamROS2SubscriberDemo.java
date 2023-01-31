package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.BigVideoPacket;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

public class WebcamROS2SubscriberDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "ROS 2 Webcam Subscriber");
   private final ImGuiPanel diagnosticPanel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
//   private ImGuiVideoPanel videoPanel;
   private RDXCVImagePanel cvImagePanel;
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

   public WebcamROS2SubscriberDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "videosub");
            ROS2Tools.createCallbackSubscription(realtimeROS2Node, ROS2Tools.BIG_VIDEO_TEST, ROS2QosProfile.BEST_EFFORT(), subscriber ->
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
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  cvImagePanel = new RDXCVImagePanel("Video1", 1920, 1080);
                  baseUI.getImGuiPanelManager().addPanel(cvImagePanel.getVideoPanel());

                  baseUI.getLayoutManager().reloadLayout();

                  messageEncodedBytePointer = new BytePointer(25000000);
                  //                  inputJPEGYUVI420Mat = new Mat(25000000);
                  inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
                  inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);

//                  ThreadTools.startAsDaemon(() ->
//                  {
//                     while (true)
//                     {
//                        transferThrottler.waitAndRun(0.005);
//                        videoSwapData.accessOnLowPriorityThread(data ->
//                        {
//                           swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(panelData ->
//                           {
//                              transferFrequencyPlot.ping();
//                              imageWidth = data.getVideoPacket().getImageWidth();
//                              imageHeight = data.getVideoPacket().getImageHeight();
//
//                              panelData.ensureTextureDimensions(imageWidth, imageHeight);
//
//                              copyBytesDurationPlot.start();
//                              BytePointer bytePointer = panelData.getRGBA8Mat().ptr(0);
//                              for (int i = 0; i < data.getVideoPacket().getData().size(); i++)
//                              {
//                                 bytePointer.put(i, data.getVideoPacket().getData().get(i));
//                              }
//                              copyBytesDurationPlot.stop();
//                           });
//                        });
//                     }
//                  }, "Transfer");
               }

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
            }

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
      if (nativesLoadedActivator.peek())
      {
         ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
         receiveFrequencyPlot.renderImGuiWidgets();
         delayPlot.renderImGuiWidgets();
         decodeDurationPlot.renderImGuiWidgets();
//         transferFrequencyPlot.renderImGuiWidgets();
//         copyBytesDurationPlot.renderImGuiWidgets();
//         uiUpdateFrequencyPlot.renderImGuiWidgets();
      }
   }

   public static void main(String[] args)
   {
      new WebcamROS2SubscriberDemo();
   }
}
