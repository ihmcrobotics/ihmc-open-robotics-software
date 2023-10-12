package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImInt;
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
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXBytedecoImagePanel;
import us.ihmc.rdx.imgui.ImGuiDirectory;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Paths;

/**
 * This can be embedded in applications to browse HDF5 files with compressed images in them.
 */
public class RDXHDF5ImageBrowser
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiDirectory logDirectory;
   private final RDXPanel panel = new RDXPanel("HDF5 Browsing", this::renderImGuiWidgets);
   private final RDXBytedecoImagePanel imagePanel;
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
   private final ImInt imageIndexToDelete = new ImInt();

   public RDXHDF5ImageBrowser()
   {
      imagePanel = new RDXBytedecoImagePanel("HDF5 Image Browser", 100, 100);

      logDirectory = new ImGuiDirectory(IHMCCommonPaths.LOGS_DIRECTORY.toString(),
                                        fileName -> h5File != null && selectedFileName.equals(fileName),
                                        pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.FILE
                                                     && pathEntry.path().getFileName().toString().endsWith(RDXHDF5ImageLoggingUI.FILE_SUFFIX),
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
      if (h5File != null)
      {
         ImGui.text(openFile);
         if (ImGui.button(labels.get("Close file")))
         {
            closeHDF5File();
         }
         else
         {
            long numberOfObjects = getNumberOfImages();
            ImGui.text(String.format("Number of objects: %d", numberOfObjects));
            ImGui.text(encoding == null ? "null" : encoding);

            if (decompressionInputPointer != null)
            {
               if (ImGui.sliderScalar(labels.get("Index"), ImGuiDataType.U32, imageIndex, 0, numberOfObjects - 1, "%d"))
               {
                  loadDatasetImage();
               }
               ImGuiTools.volatileInputInt(labels.get("Index to delete"), imageIndexToDelete, 1);
               ImGui.sameLine();
               if (ImGui.button(labels.get("Delete")))
               {
                  // TODO: How to delete?
                  LogTools.error("Not implemented!");
               }
            }
         }
      }

      logDirectory.renderImGuiWidgets();
   }

   private void closeHDF5File()
   {
      imageGroup._close();
      imageGroup = null;
      h5File._close();
      h5File = null;
      selectedFileName = "";
      openFile = "";
   }

   private void onHDF5FileSelected(String hdf5FileName)
   {
      if (h5File != null)
         closeHDF5File();

      selectedFileName = hdf5FileName;
      openFile = Paths.get(logDirectory.getDirectoryName(), hdf5FileName).toString();
      h5File = new H5File(openFile, hdf5.H5F_ACC_RDONLY);
      imageGroup = h5File.openGroup(RDXHDF5ImageLoggingUI.IMAGE_GROUP_NAME);
      imageIndex.set(0);

      isPNG = true;
      if (h5File.exists(RDXHDF5ImageLoggingUI.ENCODING_NAME))
      {
         BytePointer stringPointer = new BytePointer(100);
         DataSet dataSet = h5File.openDataSet(RDXHDF5ImageLoggingUI.ENCODING_NAME);
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
      loadDataSetImage((int) imageIndex.get(), decompressionOutputMat);

      // Could potentially be less complex, but would require a lot more code
      imagePanel.resize(decompressionOutputMat.cols(), decompressionOutputMat.rows(), null);
      // With some work we could probably remove this copy
      decompressionOutputMat.copyTo(imagePanel.getBytedecoImage().getBytedecoOpenCVMat());
   }

   public void loadDataSetImage(int imageIndex, Mat matToPack)
   {
      DataSet dataSet = imageGroup.openDataSet(String.valueOf(imageIndex));
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
         opencv_imgproc.cvtColor(bgrImage, matToPack, opencv_imgproc.COLOR_BGR2RGBA, 0);
      }
      else
      {
         opencv_imgcodecs.imdecode(decompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, yuv420Image);
         opencv_imgproc.cvtColor(yuv420Image, matToPack, opencv_imgproc.COLOR_YUV2RGBA_I420);
      }
   }

   public void destroy()
   {
      if (h5File != null)
         closeHDF5File();
   }

   public RDXBytedecoImagePanel getImagePanel()
   {
      return imagePanel;
   }

   public RDXPanel getControlPanel()
   {
      return panel;
   }

   public boolean getDataSetIsOpen()
   {
      return h5File != null;
   }

   public int getNumberOfImages()
   {
      return (int) imageGroup.getNumObjs();
   }
}
