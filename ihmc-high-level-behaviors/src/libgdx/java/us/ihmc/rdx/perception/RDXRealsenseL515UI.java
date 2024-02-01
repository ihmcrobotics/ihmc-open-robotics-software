package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.librealsense2.global.realsense2;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableRealsenseL515;
import us.ihmc.tools.time.FrequencyCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.nio.ByteOrder;

public class RDXRealsenseL515UI
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F1121365");
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXInteractableRealsenseL515 l515Interactable;
   private YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private RealsenseDeviceManager realsenseDeviceManager;
   private RealsenseDevice l515;
   private RDXBytedecoImagePanel depthImagePanel;
   private RDXBytedecoImagePanel colorImagePanel;
   private Mat depthU16C1Image;
   private Mat colorRGBImageMat;
   private BytedecoImage colorRGBImage;
   private BytedecoImage depth32FC1Image;
   private FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat laserPower = new ImFloat(100.0f);
   private final ImFloat receiverSensitivity = new ImFloat(0.5f);
   private final ImInt digitalGain = new ImInt(realsense2.RS2_DIGITAL_GAIN_LOW);
   private final String[] digitalGains = new String[] {"AUTO", "LOW", "HIGH"};
   private static final int RGBA8888_WHITE = (255 << 24) | (255 << 16) | (255 << 8) | 255;
   private RDXPointCloudRenderer pointCloudRenderer;
   private OpenCLManager openCLManager;
   private final FramePoint3D framePoint = new FramePoint3D();
   private final ImFloat focalLength = new ImFloat();
   private final ImFloat cmosWidth = new ImFloat();
   private final ImFloat cmosHeight = new ImFloat();
   private OpenCLFloatBuffer parametersBuffer;
   private _cl_program openCLProgram;
   private _cl_kernel createPointCloudKernel;
   private OpenCLFloatBuffer pointCloudRenderingBuffer;
   private int numberOfDepthPoints;

   public RDXRealsenseL515UI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RDXPanel panel = new RDXPanel("L515", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            l515Interactable = new RDXInteractableRealsenseL515(baseUI.getPrimary3DPanel());

            realsenseDeviceManager = new RealsenseDeviceManager(yoRegistry, yoGraphicsListRegistry);

            l515 = realsenseDeviceManager.createFullFeaturedL515(SERIAL_NUMBER);
            l515.enableColor(1280, 720, 30);
            l515.initialize();

            openCLManager = new OpenCLManager();
            pointCloudRenderer = new RDXPointCloudRenderer();
         }

         @Override
         public void render()
         {
            if (l515.readFrameData())
            {
               l515.updateDataBytePointers();

               if (depthImagePanel == null)
               {
                  MutableBytePointer depthFrameData = l515.getDepthFrameData();
                  depthU16C1Image = new Mat(l515.getDepthHeight(), l515.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);

                  depth32FC1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_32FC1);
                  depthImagePanel = new RDXBytedecoImagePanel("L515 Depth", l515.getDepthWidth(), l515.getDepthHeight());
                  baseUI.getImGuiPanelManager().addPanel(depthImagePanel.getImagePanel());

                  //                     colorRGBImageMat = new Mat(l515.getColorHeight(), l515.getColorWidth(), opencv_core.CV_8UC3, l515.getColorFrameData());
                  colorRGBImage = new BytedecoImage(l515.getColorWidth(), l515.getColorHeight(), opencv_core.CV_8UC3);
                  colorImagePanel = new RDXBytedecoImagePanel("L515 Color", l515.getColorWidth(), l515.getColorHeight());
                  baseUI.getImGuiPanelManager().addPanel(colorImagePanel.getImagePanel());

                  baseUI.getLayoutManager().reloadLayout();

                  numberOfDepthPoints = l515.getDepthWidth() * l515.getDepthHeight();
                  int pointCloudBytesLength = pointCloudRenderer.getFloatsPerVertex();

                  parametersBuffer = new OpenCLFloatBuffer(23);

                  openCLProgram = openCLManager.loadProgram("RealsenseL515Fusion");
                  createPointCloudKernel = openCLManager.createKernel(openCLProgram, "createPointCloud");

                  pointCloudRenderer.create(numberOfDepthPoints);
                  baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, RDXSceneLevel.MODEL);

                  pointCloudRenderingBuffer = new OpenCLFloatBuffer(numberOfDepthPoints * 8, pointCloudRenderer.getVertexBuffer());
                  depth32FC1Image.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
                  //                     colorRGBImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
                  colorImagePanel.getBytedecoImage().createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
                  pointCloudRenderingBuffer.createOpenCLBufferObject(openCLManager);
                  parametersBuffer.createOpenCLBufferObject(openCLManager);

                  focalLength.set((float) l515.getDepthFocalLengthPixelsX());
                  cmosWidth.set((float) l515.getDepthPrincipalOffsetXPixels());
                  cmosHeight.set((float) l515.getDepthPrincipalOffsetYPixels());
               }

               frameReadFrequency.ping();
               depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthDiscretization(), 0.0);

               depthImagePanel.drawDepthImage(depth32FC1Image.getBytedecoOpenCVMat());

               colorRGBImage.changeAddress(l515.getColorFrameData().address());
               //                  colorRGBImageMat.copyTo(colorRGBImage.getBytedecoOpenCVMat());

               //                  opencv_imgproc.cvtColor(color8UC3Image, colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);
               opencv_imgproc.cvtColor(colorRGBImage.getBytedecoOpenCVMat(),
                                       colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat(),
                                       opencv_imgproc.COLOR_RGB2RGBA);
               colorImagePanel.draw();

               CameraIntrinsics depthCameraIntrinsics = l515.getDepthCameraIntrinsics();

               double cmosSensorDiagonal = 0.004233; // format 1/6"
               double cmosWidthLocal = cmosWidth.get();
               double cmosHeightLocal = cmosHeight.get();
               double halfCMOSWidth = cmosWidthLocal / 2.0;
               double halfCMOSHeight = cmosHeightLocal / 2.0;

               //                  double focalLength = 0.00188; // 1.88 focal length
               double focalLengthLocal = focalLength.get();
               double focalPointToSensorEdgeX = 0.0026339;
               double focalPointToSensorEdgeY = 0.0021474;

               double cmosToPixelsX = l515.getColorWidth() / cmosWidthLocal;
               double cmosToPixelsY = l515.getColorHeight() / cmosHeightLocal;

               RigidBodyTransform transformToWorldFrame = l515Interactable.getInteractableFrameModel().getReferenceFrame().getTransformToWorldFrame();

               parametersBuffer.getBytedecoFloatBufferPointer().put(0, focalLength.get());
               parametersBuffer.getBytedecoFloatBufferPointer().put(1, cmosWidth.get());
               parametersBuffer.getBytedecoFloatBufferPointer().put(2, cmosHeight.get());
               parametersBuffer.getBytedecoFloatBufferPointer().put(3, transformToWorldFrame.getTranslation().getX32());
               parametersBuffer.getBytedecoFloatBufferPointer().put(4, transformToWorldFrame.getTranslation().getY32());
               parametersBuffer.getBytedecoFloatBufferPointer().put(5, transformToWorldFrame.getTranslation().getZ32());
               parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) transformToWorldFrame.getRotation().getM00());
               parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) transformToWorldFrame.getRotation().getM01());
               parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) transformToWorldFrame.getRotation().getM02());
               parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) transformToWorldFrame.getRotation().getM10());
               parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) transformToWorldFrame.getRotation().getM11());
               parametersBuffer.getBytedecoFloatBufferPointer().put(11, (float) transformToWorldFrame.getRotation().getM12());
               parametersBuffer.getBytedecoFloatBufferPointer().put(12, (float) transformToWorldFrame.getRotation().getM20());
               parametersBuffer.getBytedecoFloatBufferPointer().put(13, (float) transformToWorldFrame.getRotation().getM21());
               parametersBuffer.getBytedecoFloatBufferPointer().put(14, (float) transformToWorldFrame.getRotation().getM22());
               parametersBuffer.getBytedecoFloatBufferPointer().put(15, (float) depthCameraIntrinsics.getCx());
               parametersBuffer.getBytedecoFloatBufferPointer().put(16, (float) depthCameraIntrinsics.getCy());
               parametersBuffer.getBytedecoFloatBufferPointer().put(17, (float) depthCameraIntrinsics.getFx());
               parametersBuffer.getBytedecoFloatBufferPointer().put(18, (float) depthCameraIntrinsics.getFy());
               parametersBuffer.getBytedecoFloatBufferPointer().put(19, (float) l515.getDepthWidth());
               parametersBuffer.getBytedecoFloatBufferPointer().put(20, (float) l515.getDepthHeight());
               parametersBuffer.getBytedecoFloatBufferPointer().put(21, (float) l515.getColorWidth());
               parametersBuffer.getBytedecoFloatBufferPointer().put(22, (float) l515.getColorHeight());

               depth32FC1Image.writeOpenCLImage(openCLManager);
               //                     colorRGBImage.writeOpenCLImage(openCLManager);
               colorImagePanel.getBytedecoImage().writeOpenCLImage(openCLManager);
               parametersBuffer.writeOpenCLBufferObject(openCLManager);

               openCLManager.setKernelArgument(createPointCloudKernel, 0, depth32FC1Image.getOpenCLImageObject());
               openCLManager.setKernelArgument(createPointCloudKernel, 1, colorImagePanel.getBytedecoImage().getOpenCLImageObject());
               openCLManager.setKernelArgument(createPointCloudKernel, 2, pointCloudRenderingBuffer.getOpenCLBufferObject());
               openCLManager.setKernelArgument(createPointCloudKernel, 3, parametersBuffer.getOpenCLBufferObject());
               openCLManager.execute2D(createPointCloudKernel, l515.getDepthWidth(), l515.getDepthHeight());
               pointCloudRenderingBuffer.readOpenCLBufferObject(openCLManager);

               pointCloudRenderer.updateMeshFastest(numberOfDepthPoints);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (depthImagePanel != null)
            {
               ImGui.text("Depth frame data size: " + l515.getDepthFrameDataSize());
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
               ImGui.text("Depth to meters conversion: " + l515.getDepthDiscretization());

               volatileInputFloat(labels.get("Focal length"), focalLength, 0.00001f);
               volatileInputFloat(labels.get("CMOS width"), cmosWidth, 0.00001f);
               volatileInputFloat(labels.get("CMOS height"), cmosHeight, 0.00001f);

               if (ImGui.sliderFloat(labels.get("Laser power"), laserPower.getData(), 0.0f, 100.0f))
               {
                  l515.setLaserPower(laserPower.get());
               }

               if (ImGui.combo(labels.get("Digital gain"), digitalGain, digitalGains))
               {
                  if (digitalGain.get() == 0)
                  {
                     l515.setDigitalGain(realsense2.RS2_DIGITAL_GAIN_AUTO);
                  }
                  else if (digitalGain.get() == 1)
                  {
                     l515.setDigitalGain(realsense2.RS2_DIGITAL_GAIN_LOW);
                  }
                  else
                  {
                     l515.setDigitalGain(realsense2.RS2_DIGITAL_GAIN_HIGH);
                  }
               }

               ImGui.text("Unsigned 16 Depth:");

               for (int i = 0; i < 5; i++)
               {
                  ImGui.text(depthU16C1Image.ptr(0, i).getShort() + " ");
               }

               ImGui.text("Float 32 Meters:");

               depth32FC1Image.rewind();
               for (int i = 0; i < 5; i++)
               {
                  ImGui.text(depth32FC1Image.getBackingDirectByteBuffer().getFloat() + " ");
               }

               ImGui.text("Depth R G B A:");

               depthImagePanel.getBytedecoImage().rewind();
               for (int i = 0; i < 5; i++)
               {
                  printBytes(depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                             depthImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
               }

               ImGui.text("Color R G B:");

               for (int i = 0; i < 5; i++)
               {
                  printBytes(colorRGBImage.getBytedecoOpenCVMat().ptr(0, i).get(0),
                             colorRGBImage.getBytedecoOpenCVMat().ptr(0, i).get(1),
                             colorRGBImage.getBytedecoOpenCVMat().ptr(0, i).get(2));
               }

               ImGui.text("Color R G B A:");

               colorImagePanel.getBytedecoImage().rewind();
               for (int i = 0; i < 5; i++)
               {
                  printBytes(colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(0),
                             colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(1),
                             colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(2),
                             colorImagePanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(0, i).get(3));
               }
            }
         }

         private boolean volatileInputFloat(String label, ImFloat imFloat, float step)
         {
            int inputTextFlags = ImGuiInputTextFlags.None;
            inputTextFlags += ImGuiInputTextFlags.EnterReturnsTrue;
            return ImGui.inputFloat(label, imFloat, step, 0, "%.6f", inputTextFlags);
         }

         private void printBytes(byte byte0, byte byte1, byte byte2, byte byte3)
         {
            printInts(Byte.toUnsignedInt(byte0), Byte.toUnsignedInt(byte1), Byte.toUnsignedInt(byte2), Byte.toUnsignedInt(byte3));
         }

         private void printBytes(byte byte0, byte byte1, byte byte2)
         {
            printInts(Byte.toUnsignedInt(byte0), Byte.toUnsignedInt(byte1), Byte.toUnsignedInt(byte2));
         }

         private void printInts(int int0, int int1, int int2, int int3)
         {
            ImGui.text(int0 + " " + int1 + " " + int2 + " " + int3);
         }

         private void printInts(int int0, int int1, int int2)
         {
            ImGui.text(int0 + " " + int1 + " " + int2);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            openCLManager.destroy();
            l515.deleteDevice();
            realsenseDeviceManager.deleteContext();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRealsenseL515UI();
   }
}
