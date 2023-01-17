package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.CharPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;

import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

public class WebcamHDF5LoggingDemo
{
   public static final String FILE_SUFFIX = "Images" + HDF5Tools.HDF5_FILE_EXTENSION;
   public static final String IMAGE_GROUP_NAME = "image";
   public static final String ENCODING_NAME = "encoding";

   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Webcam HDF5 Logging Demo");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXOpenCVWebcamReader webcamReader;
   private Mat bgrWebcamCopy;
   private volatile boolean running = true;
   private H5File h5File = null;
   private final ImString logDirectory = new ImString(IHMCCommonPaths.LOGS_DIRECTORY.toString());
   private String logFile;
   private final ImPlotStopwatchPlot encodeDurationPlot = new ImPlotStopwatchPlot("Encode duration");
   private final ImPlotIntegerPlot compressedBytesPlot = new ImPlotIntegerPlot("Compressed bytes");
   private final ImInt jpegQuality = new ImInt(95);
   private IntPointer pngCompressionParameters;
   private IntPointer jpegCompressionParameters;
   private Mat yuv420Image;
   private BytePointer compressedImageBuffer;
   private DataType nativeByteType;
   private Group imageGroup;
   private long imageIndex;
   private final Object syncObject = new Object();

   private enum Encoding { PNG, JPEG }
   private Encoding encodingSelection = Encoding.PNG;

   public WebcamHDF5LoggingDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(new ImGuiPanel("Logging", this::renderImGuiWidgets));
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            webcamReader = new RDXOpenCVWebcamReader(nativesLoadedActivator);
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getStatisticsPanel());
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  webcamReader.create();
                  baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

                  pngCompressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
                  jpegCompressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, jpegQuality.get());
                  yuv420Image = new Mat();
                  compressedImageBuffer = new BytePointer((long) webcamReader.getImageWidth() * webcamReader.getImageHeight() * 3); // BGR8
                  bgrWebcamCopy = new Mat();
                  nativeByteType = new DataType(PredType.NATIVE_B8());

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        webcamReader.readWebcamImage();

                        synchronized (syncObject)
                        {
                           webcamReader.getBGRImage().copyTo(bgrWebcamCopy);
                        }
                     }
                  }, "CameraRead");
               }

               webcamReader.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            running = false;
            webcamReader.dispose();
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      if (nativesLoadedActivator.peek())
      {
         encodeDurationPlot.renderImGuiWidgets();
         compressedBytesPlot.renderImGuiWidgets();
      }

      if (h5File == null)
      {
         ImGuiTools.inputText(labels.get("Log directory"), logDirectory);

         ImGui.text("Encoding");
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("PNG"), encodingSelection == Encoding.PNG))
         {
            encodingSelection = Encoding.PNG;
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("JPEG"), encodingSelection == Encoding.JPEG))
         {
            encodingSelection = Encoding.JPEG;
         }
         if (encodingSelection == Encoding.JPEG)
         {
            if (ImGui.sliderInt(labels.get("JPEG Quality"), jpegQuality.getData(), 0, 100))
            {
               jpegCompressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, jpegQuality.get());
            }
         }

         if (ImGui.button(labels.get("Begin")))
         {
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
            String logFileName = dateFormat.format(new Date()) + "_" + FILE_SUFFIX;
            FileTools.ensureDirectoryExists(Paths.get(logDirectory.get()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            logFile = Paths.get(logDirectory.get(), logFileName).toString();
            h5File = new H5File(logFile, hdf5.H5F_ACC_TRUNC);

            CharPointer encodingPointer = new CharPointer(encodingSelection.name().toLowerCase());
            int rank = 1;
            long[] dimensions = { encodingPointer.limit() };
            DataSpace dataSpace = new DataSpace(rank, dimensions);
            DataType dataType = new DataType(PredType.NATIVE_CHAR());
            DataSet dataSet = h5File.createDataSet(ENCODING_NAME, dataType, dataSpace);
            dataSet.write(encodingPointer, dataType);
            dataSet.close();
            dataSpace.close();

            imageGroup = h5File.createGroup(IMAGE_GROUP_NAME);
            imageIndex = 0;
         }
      }
      else
      {
         ImGui.text(logFile);
         if (ImGui.button(labels.get("Close file")))
         {
            imageGroup.close();
            imageGroup = null;
            h5File.close();
            h5File = null;
         }

         if (ImGui.button("Capture image"))
         {
            synchronized (syncObject)
            {
               encodeDurationPlot.start();
               if (encodingSelection == Encoding.PNG)
               {
                  opencv_imgcodecs.imencode(".png", bgrWebcamCopy, compressedImageBuffer, pngCompressionParameters);
               }
               else
               {
                  opencv_imgproc.cvtColor(bgrWebcamCopy, yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
                  opencv_imgcodecs.imencode(".jpg", yuv420Image, compressedImageBuffer, jpegCompressionParameters);
               }
               encodeDurationPlot.stop();
            }

            compressedBytesPlot.addValue((int) compressedImageBuffer.limit());

            int rank = 1;
            long[] dimensions = {compressedImageBuffer.limit() };
            DataSpace dataSpace = new DataSpace(rank, dimensions);
            DataSet dataSet = imageGroup.createDataSet(String.valueOf(imageIndex), nativeByteType, dataSpace);
            dataSet.write(compressedImageBuffer, nativeByteType);
            dataSet.close();
            dataSpace.close();

            ++imageIndex;
         }
      }
   }

   public static void main(String[] args)
   {
      new WebcamHDF5LoggingDemo();
   }
}
