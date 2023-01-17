package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImLong;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.tools.ImGuiDirectory;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.Activator;

import java.nio.file.Paths;

public class HDF5ImageBrowserDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "HDF5 Image Browser Demo");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiDirectory logDirectory;
   private RDXCVImagePanel imagePanel;
   private H5File h5File = null;
   private String selectedFileName = "";
   private String openFile = "";
   private Group imageGroup;
   private final ImLong imageIndex = new ImLong();
   private DataType nativeBytesType;
   private BytePointer pngBytesPointer;
   private ShortPointer shortPointerForReadingHDF5;
   private Mat decompressionInputMat;
   private Mat bgrImage;

   public HDF5ImageBrowserDemo()
   {
      logDirectory = new ImGuiDirectory(IHMCCommonPaths.LOGS_DIRECTORY.toString(),
                                        fileName -> h5File != null && selectedFileName.equals(fileName),
                                        pathEntry -> pathEntry.type() == BasicPathVisitor.PathType.FILE
                                                     && pathEntry.path().getFileName().toString().endsWith(WebcamHDF5LoggingDemo.FILE_SUFFIX),
                                        this::onHDF5FileSelected);

      baseUI.getImGuiPanelManager().addPanel(new ImGuiPanel("HDF5 Browsing", this::renderImGuiWidgets));
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  imagePanel = new RDXCVImagePanel("Image Monitor", 1920, 1080);
                  baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

                  pngBytesPointer = new BytePointer(1920 * 1080 * 3);
                  shortPointerForReadingHDF5 = new ShortPointer(pngBytesPointer);
                  nativeBytesType = new DataType(PredType.NATIVE_B8());

                  decompressionInputMat = new Mat(1, 1, opencv_core.CV_8UC1);
                  bgrImage = new Mat(1080, 1920, opencv_core.CV_8UC3);
               }

               imagePanel.draw();
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

            if (pngBytesPointer != null)
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
      if (bgrImage != null)
      {
         selectedFileName = hdf5FileName;
         openFile = Paths.get(logDirectory.getDirectoryName(), hdf5FileName).toString();
         h5File = new H5File(openFile, hdf5.H5F_ACC_RDONLY);
         imageGroup = h5File.openGroup(WebcamHDF5LoggingDemo.IMAGE_GROUP_NAME);
         imageIndex.set(0);
         loadDatasetImage();
      }
   }

   private void loadDatasetImage()
   {
      DataSet dataSet = imageGroup.openDataSet(String.valueOf(imageIndex.get()));
      dataSet.read(shortPointerForReadingHDF5, nativeBytesType);
      pngBytesPointer.limit(shortPointerForReadingHDF5.limit());

      decompressionInputMat.cols((int) pngBytesPointer.limit());
      decompressionInputMat.data(pngBytesPointer);

      opencv_imgcodecs.imdecode(decompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, bgrImage);
      opencv_imgproc.cvtColor(bgrImage, imagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
   }

   public static void main(String[] args)
   {
      new HDF5ImageBrowserDemo();
   }
}
