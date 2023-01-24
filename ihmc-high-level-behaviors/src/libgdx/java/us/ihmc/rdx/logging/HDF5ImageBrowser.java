package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImLong;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXCVImagePanel;
import us.ihmc.rdx.ui.tools.ImGuiDirectory;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Paths;

public class HDF5ImageBrowser
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiDirectory logDirectory;
   private final ImGuiPanel panel = new ImGuiPanel("HDF5 Browsing", this::renderImGuiWidgets);
   private final RDXCVImagePanel imagePanel;
   private H5File h5File = null;
   private String selectedFileName = "";
   private String openFile = "";
   private Group imageGroup;
   private final ImLong imageIndex = new ImLong();
   private final DataType nativeBytesType;
   private BytePointer decompressionInputPointer;
   private final Mat decompressionInputMat;
   private final Mat bgrImage;
   private final Mat yuv420Image;
   private final Mat decompressionOutputMat;
   private boolean isPNG;
   private String encoding;

   public HDF5ImageBrowser()
   {
      imagePanel = new RDXCVImagePanel("HDF5 Image Browser", 1, 1);

      logDirectory = new ImGuiDirectory(IHMCCommonPaths.LOGS_DIRECTORY.toString(),
                                        fileName -> h5File != null && selectedFileName.equals(fileName),
                                        pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.FILE
                                                     && pathEntry.path().getFileName().toString().endsWith(HDF5ImageLogging.FILE_SUFFIX),
                                        this::onHDF5FileSelected);

      nativeBytesType = new DataType(PredType.NATIVE_B8());

      decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
      yuv420Image = new Mat(1, 1, opencv_core.CV_8UC1);
      bgrImage = new Mat(1, 1, opencv_core.CV_8UC3);
      decompressionOutputMat = new Mat(1, 1, opencv_core.CV_8UC4);

      if (h5File != null)
      {
         loadDatasetImage();
      }
   }

   public void update()
   {
      imagePanel.draw();
   }

   private void renderImGuiWidgets()
   {
      if (h5File == null)
      {
         logDirectory.renderImGuiWidgets();
      }
      else
      {
         ImGui.text(openFile);
         if (ImGui.button(labels.get("Close file")))
         {
            imageGroup.close();
            imageGroup = null;
            h5File.close();
            h5File = null;
            selectedFileName = "";
            openFile = "";
         }
         else
         {
            long numberOfObjects = imageGroup.getNumObjs();
            ImGui.text(String.format("Number of objects: %d", numberOfObjects));
            ImGui.text(encoding == null ? "null" : encoding);

            if (decompressionInputPointer != null)
            {
               if (ImGui.sliderScalar(labels.get("Index"), ImGuiDataType.U32, imageIndex, 0, numberOfObjects - 1, "%d"))
               {
                  loadDatasetImage();
               }
            }
         }
      }
   }

   private void onHDF5FileSelected(String hdf5FileName)
   {
      selectedFileName = hdf5FileName;
      openFile = Paths.get(logDirectory.getDirectoryName(), hdf5FileName).toString();
      h5File = new H5File(openFile, hdf5.H5F_ACC_RDONLY);
      imageGroup = h5File.openGroup(HDF5ImageLogging.IMAGE_GROUP_NAME);
      imageIndex.set(0);

      isPNG = true;
      if (h5File.exists(HDF5ImageLogging.ENCODING_NAME))
      {
         BytePointer stringPointer = new BytePointer(100);
         DataSet dataSet = h5File.openDataSet(HDF5ImageLogging.ENCODING_NAME);
         dataSet.read(stringPointer, new DataType(PredType.NATIVE_CHAR())); // intentionally reads a string
         encoding = stringPointer.getString();
         isPNG = encoding.equals("png");
      }

      if (bgrImage != null)
      {
         loadDatasetImage();
      }
   }

   private void loadDatasetImage()
   {
      DataSet dataSet = imageGroup.openDataSet(String.valueOf(imageIndex.get()));
      long inMemDataSize = dataSet.getInMemDataSize();
      if (decompressionInputPointer == null || inMemDataSize > decompressionInputPointer.capacity())
      {
         decompressionInputPointer = new BytePointer(2 * inMemDataSize); // Allocate 2x so we aren't always doing this
      }
      dataSet.read((Pointer) decompressionInputPointer, nativeBytesType);

      decompressionInputMat.cols((int) decompressionInputPointer.limit());
      decompressionInputMat.data(decompressionInputPointer);

      if (isPNG)
      {
         opencv_imgcodecs.imdecode(decompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, bgrImage);
         opencv_imgproc.cvtColor(bgrImage, decompressionOutputMat, opencv_imgproc.COLOR_BGR2RGBA, 0);
      }
      else
      {
         opencv_imgcodecs.imdecode(decompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, yuv420Image);
         opencv_imgproc.cvtColor(yuv420Image, decompressionOutputMat, opencv_imgproc.COLOR_YUV2RGBA_I420);
      }

      // Could potentially be less complex, but would require a lot more code
      imagePanel.resize(decompressionOutputMat.cols(), decompressionOutputMat.rows(), null);
      // With some work we could probably remove this copy
      decompressionOutputMat.copyTo(imagePanel.getBytedecoImage().getBytedecoOpenCVMat());
   }

   public RDXCVImagePanel getImagePanel()
   {
      return imagePanel;
   }

   public ImGuiPanel getControlPanel()
   {
      return panel;
   }
}
