package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImPlotIntegerPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This can be embedded in applications to record compressed images to HDF5 files.
 */
public class RDXHDF5ImageLoggingUI
{
   public static final String FILE_SUFFIX = "Images" + PerceptionLoggerConstants.HDF5_FILE_EXTENSION;
   public static final String IMAGE_GROUP_NAME = "image";
   public static final String ENCODING_NAME = "encoding";

   private final Mat bgrSourceCopy;
   private final Object syncObject = new Object();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXPanel panel = new RDXPanel("Logging", this::renderImGuiWidgets);
   private H5File h5File = null;
   private final ImString logDirectory = new ImString(IHMCCommonPaths.LOGS_DIRECTORY.toString());
   private String logFile;
   private final ImPlotStopwatchPlot encodeDurationPlot = new ImPlotStopwatchPlot("Encode duration");
   private final ImPlotIntegerPlot compressedBytesPlot = new ImPlotIntegerPlot("Compressed bytes");

   private enum Encoding {PNG, JPEG}

   private Encoding encodingSelection = Encoding.JPEG;
   private final ImInt jpegQuality = new ImInt(95);
   private final IntPointer pngCompressionParameters;
   private IntPointer jpegCompressionParameters;
   private final Mat yuv420Image;
   private final BytePointer compressedImageBuffer;
   private final DataType nativeByteType;
   private Group imageGroup;
   private long imageIndex;

   public RDXHDF5ImageLoggingUI(int imageWidth, int imageHeight)
   {
      pngCompressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
      jpegCompressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, jpegQuality.get());
      yuv420Image = new Mat();
      compressedImageBuffer = new BytePointer((long) imageWidth * imageHeight * 3); // BGR8
      bgrSourceCopy = new Mat();
      nativeByteType = new DataType(PredType.NATIVE_B8());
   }

   public void copyBGRImage(Mat bgrImageToCopy)
   {
      synchronized (syncObject)
      {
         bgrImageToCopy.copyTo(bgrSourceCopy);
      }
   }

   public void copyBayerBGImage(Mat bayerBGImageToCopy)
   {
      synchronized (syncObject)
      {
         opencv_imgproc.cvtColor(bayerBGImageToCopy, bgrSourceCopy, opencv_imgproc.COLOR_BayerBG2BGR);
      }
   }

   private void renderImGuiWidgets()
   {
      encodeDurationPlot.renderImGuiWidgets();
      compressedBytesPlot.renderImGuiWidgets();

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

            BytePointer encodingNameBytes = encodingSelection == Encoding.PNG ? new BytePointer("png".getBytes()) : new BytePointer("jpeg".getBytes());
            int rank = 1;
            long[] dimensions = {encodingNameBytes.limit()};
            DataSpace dataSpace = new DataSpace(rank, dimensions);
            DataType dataType = new DataType(PredType.NATIVE_CHAR());
            DataSet dataSet = h5File.createDataSet(ENCODING_NAME, dataType, dataSpace);
            dataSet.write(encodingNameBytes, dataType); // intentionally writes a string
            dataSet._close();
            dataSpace._close();

            imageGroup = h5File.createGroup(IMAGE_GROUP_NAME);
            imageIndex = 0;
         }
      }
      else
      {
         ImGui.text(logFile);
         if (ImGui.button(labels.get("Close file")))
         {
            closeHDF5File();
         }

         if (ImGui.button("Capture image"))
         {
            synchronized (syncObject)
            {
               encodeDurationPlot.start();
               if (encodingSelection == Encoding.PNG)
               {
                  opencv_imgcodecs.imencode(".png", bgrSourceCopy, compressedImageBuffer, pngCompressionParameters);
               }
               else
               {
                  opencv_imgproc.cvtColor(bgrSourceCopy, yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
                  opencv_imgcodecs.imencode(".jpg", yuv420Image, compressedImageBuffer, jpegCompressionParameters);
               }
               encodeDurationPlot.stop();
            }

            compressedBytesPlot.addValue((int) compressedImageBuffer.limit());

            int rank = 1;
            long[] dimensions = {compressedImageBuffer.limit()};
            DataSpace dataSpace = new DataSpace(rank, dimensions);
            DataSet dataSet = imageGroup.createDataSet(String.valueOf(imageIndex), nativeByteType, dataSpace);
            dataSet.write((Pointer) compressedImageBuffer, nativeByteType);
            dataSet._close();
            dataSpace._close();

            ++imageIndex;
         }
      }
   }

   private void closeHDF5File()
   {
      imageGroup._close();
      imageGroup = null;
      h5File._close();
      h5File = null;
   }

   public void destroy()
   {
      if (h5File != null)
         closeHDF5File();
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
