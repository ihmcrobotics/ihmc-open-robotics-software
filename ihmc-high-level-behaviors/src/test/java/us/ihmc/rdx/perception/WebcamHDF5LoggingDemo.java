package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImString;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.rdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.rdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;

import java.io.File;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

public class WebcamHDF5LoggingDemo
{
   public static final String FILE_SUFFIX = "Images" + HDF5Tools.HDF5_FILE_EXTENSION;
   public static final String IMAGE_GROUP_NAME = "image";

   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Webcam HDF5 Logging Demo");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXOpenCVWebcamReader webcamReader;
//   private Mat bgrWebcamCopy;
   private BytedecoImage bgrWebcamCopy;
   private volatile boolean running = true;
   private HDF5Manager hdf5Manager = null;
   private final ImString logDirectory = new ImString(IHMCCommonPaths.LOGS_DIRECTORY.toString());
   private String logFile;
   private final ImPlotStopwatchPlot encodeDurationPlot = new ImPlotStopwatchPlot("Encode duration");
   private final ImPlotFrequencyPlot publishFrequencyPlot = new ImPlotFrequencyPlot("Publish frequency");
   private final ImPlotIntegerPlot compressedBytesPlot = new ImPlotIntegerPlot("Compressed bytes");
   private IntPointer compressionParameters;
   private BytePointer pngImageBuffer;
   private Group imageGroup;
   private long imageIndex;

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

                  compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
                  pngImageBuffer = new BytePointer((long) webcamReader.getImageWidth() * webcamReader.getImageHeight() * 3); // BGR8
//                  bgrWebcamCopy = new Mat();
                  bgrWebcamCopy = new BytedecoImage(1920, 1080, opencv_core.CV_8UC3);

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        webcamReader.readWebcamImage();

                        synchronized (this)
                        {
//                           webcamReader.getBGRImage().copyTo(bgrWebcamCopy);
                           webcamReader.getBGRImage().copyTo(bgrWebcamCopy.getBytedecoOpenCVMat());
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
         publishFrequencyPlot.renderImGuiWidgets();
         encodeDurationPlot.renderImGuiWidgets();
         compressedBytesPlot.renderImGuiWidgets();
      }

      if (hdf5Manager == null)
      {
         ImGuiTools.inputText(labels.get("Log directory"), logDirectory);

         // TODO: Radio buttons for PNG, JPEG

         if (ImGui.button(labels.get("Begin")))
         {
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
            String logFileName = dateFormat.format(new Date()) + "_" + FILE_SUFFIX;
            FileTools.ensureDirectoryExists(Paths.get(logDirectory.get()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            logFile = Paths.get(logDirectory.get(), logFileName).toString();
            hdf5Manager = new HDF5Manager(logFile, hdf5.H5F_ACC_TRUNC);

            imageGroup = hdf5Manager.getGroup(IMAGE_GROUP_NAME);
            imageIndex = 0;
         }
      }
      else
      {
         ImGui.text(logFile);
         if (ImGui.button(labels.get("Close file")))
         {
            hdf5Manager.closeFile();
            hdf5Manager = null;
         }

         if (ImGui.button("Capture image"))
         {
            synchronized (this)
            {
//               opencv_imgcodecs.imencode(".png", bgrWebcamCopy, pngImageBuffer, compressionParameters);
            HDF5Tools.storeBytesFromPointer(imageGroup, imageIndex, bgrWebcamCopy.getBytedecoByteBufferPointer(), bgrWebcamCopy.getBytedecoByteBufferPointer().limit());
            }

//            HDF5Tools.storeBytesFromPointer(imageGroup, imageIndex, pngImageBuffer, pngImageBuffer.limit());

//            int rank = 1;
//            long[] dimensions = { pngImageBuffer.limit() };
//            DataSpace dataSpace = new DataSpace(rank, dimensions);
//            DataType dataType = new DataType(PredType.NATIVE_B8());
//            DataSet dataSet = imageGroup.createDataSet(String.valueOf(imageIndex), dataType, dataSpace);
//            dataSet.write(pngImageBuffer, dataType);
//            dataSet.close();
//            dataSpace.close();

            ++imageIndex;
         }
      }
   }

   public static void main(String[] args)
   {
      new WebcamHDF5LoggingDemo();
   }
}
