package us.ihmc.rdx.simulation.scs2;

import com.badlogic.gdx.graphics.Pixmap;
import imgui.internal.ImGui;
import javafx.application.Application;
import javafx.stage.Stage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.javacpp.indexer.FloatRawIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_dnn;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_dnn.Net;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.neural.YOLOv8ONNX;
import us.ihmc.perception.opencv.OpenCVArUcoMarker;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.RDXBytedecoImagePanel;
import us.ihmc.rdx.perception.RDXOpenCVArUcoMarkerDetectionUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2CenterposeVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.string.StringTools;

import java.nio.ByteBuffer;
import java.util.ArrayList;

import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2RGBA;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_RGBA2BGR;

public class RDXYOLOv8WithONNXRuntimeDemo
    {
        private final RDXBaseUI baseUI = new RDXBaseUI();
        private RDXEnvironmentBuilder environmentBuilder;
        private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
        private RDXHighLevelDepthSensorSimulator cameraSensor;
//        private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
//        private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;

        private final RDXPerceptionVisualizerPanel perceptionVisualizersUI;
        private final RDXPanel mainPanel;
        private RDXBytedecoImagePanel YOLOv8ImagePanel;
        private final ImageMessage imageMessage = new ImageMessage();
        private final SampleInfo sampleInfo = new SampleInfo();
        private Mat decompressedImage;
        private Mat rgba8Mat;
        private Mat rgb8Mat;
        private Pixmap pixmap1;
        private Pixmap pixmap2;
        private BytePointer rgba8888BytePointer;
        private BytePointer rgb888BytePointer;
        private boolean needNewTexture = false;
        private Net net;
        private Size imageSize;
        private final Scalar scalarZero = new Scalar(0.0);
        private final MatVector outputBlobs = new MatVector();

        private float confidenceThreshold = 0.1f;


        RDXROS2ImageMessageVisualizer zedLeftColorVisualizer;


   public RDXYOLOv8WithONNXRuntimeDemo()
        {

            perceptionVisualizersUI = new RDXPerceptionVisualizerPanel();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizersUI);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizersUI);
            mainPanel = new RDXPanel("YOLO Demo");

            baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
            {
                @Override
                public void create()
                {
                    baseUI.create();

                    environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
                    environmentBuilder.create();
                    baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
                    environmentBuilder.loadEnvironment("DoorsForArUcoTesting.json");

                    sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
                    sensorPoseGizmo.setResizeAutomatically(true);
                    baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
                    baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
                    baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);
                    sensorPoseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 1.0);

                    cameraSensor = RDXSimulatedSensorFactory.createBlackflyFisheyeImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
                    cameraSensor.setSensorEnabled(true);
                    cameraSensor.setRenderColorVideoDirectly(true);
                    baseUI.getImGuiPanelManager().addPanel(cameraSensor);
                    baseUI.getPrimaryScene().addRenderableProvider(cameraSensor::getRenderables);

                    baseUI.getImGuiPanelManager().addPanel(mainPanel);
                    YOLOv8ImagePanel = new RDXBytedecoImagePanel("YOLO Demo Img", 1280, 720, false);
                    mainPanel.addChild(YOLOv8ImagePanel.getImagePanel());


                    String titleBeforeAdditions = "ZED 2 Color Left";
                    DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.FAST_RTPS;
                    ROS2Topic<ImageMessage> topic = PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT);
                    RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
                    ROS2Tools.createCallbackSubscription(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT(), this::getLeftImage);
                    imageSize = new Size(640, 640);

                    WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(YOLOv8ONNX.class, "/yolo/");
                    WorkspaceFile onnxFile = new WorkspaceFile(directory, "yolov8n-seg.onnx");
                    net = opencv_dnn.readNet(onnxFile.getFilesystemFile().toString());
                    if (opencv_core.getCudaEnabledDeviceCount() > 0){
                        net.setPreferableBackend(opencv_dnn.DNN_BACKEND_CUDA);
                        net.setPreferableTarget(opencv_dnn.DNN_TARGET_CUDA);
                    }
                    else{
                        net.setPreferableBackend(opencv_dnn.DNN_BACKEND_OPENCV);
                        net.setPreferableTarget(opencv_dnn.DNN_TARGET_CPU);
                    }


                }



                private void getLeftImage(Subscriber<ImageMessage> subscriber)
                {
                    imageMessage.getData().resetQuick();
                    subscriber.takeNextData(imageMessage, sampleInfo);
                    int bytesIfUncompressed = imageMessage.getImageHeight() * imageMessage.getImageWidth() * 3;
                    ByteBuffer incomingCompressedImageBuffer = NativeMemoryTools.allocate(bytesIfUncompressed);
                    BytePointer incomingCompressedImageBytePointer = new BytePointer(incomingCompressedImageBuffer);
                    Mat compressedBytesMat = new Mat(1, 1, opencv_core.CV_8UC1);
                    decompressedImage = new Mat(imageMessage.getImageHeight(), imageMessage.getImageWidth(), opencv_core.CV_8UC3);
                    int numberOfBytes = imageMessage.getData().size();
                    incomingCompressedImageBuffer.rewind();
                    incomingCompressedImageBuffer.limit(bytesIfUncompressed);
                    for (int i = 0; i < numberOfBytes; i++)
                    { incomingCompressedImageBuffer.put(imageMessage.getData().get(i)); }
                    incomingCompressedImageBuffer.flip();
                    compressedBytesMat.cols(numberOfBytes);
                    compressedBytesMat.data(incomingCompressedImageBytePointer);
                    opencv_imgcodecs.imdecode(compressedBytesMat, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedImage);
                    updateImageDimensions(imageMessage.getImageWidth(), imageMessage.getImageHeight());
                    opencv_imgproc.cvtColor(decompressedImage, rgba8Mat, opencv_imgproc.COLOR_BGR2BGRA);
                    decompressedImage.copyTo(rgb8Mat);
                }


                public void updateYOLORender(){
                    if (rgba8Mat != null) {
                        YOLOv8ImagePanel.drawColorImage(rgba8Mat);

                        try{
                            StringVector outNames = net.getUnconnectedOutLayersNames();
                            Mat blob;
                            blob = opencv_dnn.blobFromImage(rgb8Mat, 1 / 255.0, imageSize, scalarZero, true, true, opencv_core.CV_32F);
                            net.setInput(blob);
                            outputBlobs.resize(outNames.size());

                            net.forward(outputBlobs, outNames);

                            postprocess(outputBlobs);






//                            data0.get(0,0,0);
//                            data1.get(0,0,0,0);

                        }
                        catch(Exception exception){
                            exception.printStackTrace();
                        }

                    }
                }

                public void postprocess(MatVector outputs){
                    Mat output0 = outputs.get(0);
                    FloatIndexer data0 = output0.createIndexer();
                    Mat output1 = outputs.get(1);
                    FloatIndexer data1 = output1.createIndexer();

                    for (long i = 0; i < 4800; i++) {
                        for (long j = 0; j < 80; j++) {
                            float confidence = data0.get(0,4+j, i);
                            if (confidence >= confidenceThreshold){
                                int a =1;
                            }

                        }

                    }


                }


                protected void updateImageDimensions(int imageWidth, int imageHeight)
                {
                    if (rgba8Mat == null || pixmap1.getWidth() != imageWidth || pixmap1.getHeight() != imageHeight)
                    {
                        if (pixmap1 != null)
                        {
                            pixmap1.dispose();
                        }
                        pixmap1 = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
                        rgba8888BytePointer = new BytePointer(pixmap1.getPixels());
                        rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, rgba8888BytePointer);
                        needNewTexture = true;
                    }
                    if (rgb8Mat == null || pixmap2.getWidth() != imageWidth || pixmap2.getHeight() != imageHeight)
                    {
                        if (pixmap2 != null)
                        {
                            pixmap2.dispose();
                        }
                        pixmap2 = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGB888);
                        rgb888BytePointer = new BytePointer(pixmap2.getPixels());
                        rgb8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, rgba8888BytePointer);
                        needNewTexture = true;
                    }
                }


                @Override
                public void render()
                {
                    environmentBuilder.update();
                    cameraSensor.render(baseUI.getPrimaryScene());

                    updateYOLORender();

                    baseUI.renderBeforeOnScreenUI();
                    baseUI.renderEnd();
                }

                public void renderImGuiWidgets()
                {
                    ImGui.text("Image width: " + 1280 + " height: " + 720);
                }


                @Override
                public void dispose()
                {
                    baseUI.dispose();
                    environmentBuilder.destroy();
                    cameraSensor.dispose();
                }
            });
        }

        public static void main(String[] args)
        {
            new RDXYOLOv8WithONNXRuntimeDemo();
        }
    }
