package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
import imgui.type.ImLong;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.HDF5Tools;
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
   private HDF5Manager hdf5Manager = null;
   private String selectedFileName = "";
   private String openFile = "";
   private Group imageGroup;
   private final ImLong imageIndex = new ImLong();
   private BytePointer pngBytesPointer;
   private Mat decompressionInputMat;
   private Mat bgrImage;

   public HDF5ImageBrowserDemo()
   {
      logDirectory = new ImGuiDirectory(IHMCCommonPaths.LOGS_DIRECTORY.toString(),
                                        fileName -> hdf5Manager != null && selectedFileName.equals(fileName),
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
      if (hdf5Manager == null)
      {
         logDirectory.renderImGuiWidgets();
      }
      else
      {
         ImGui.text(openFile);
         if (ImGui.button(labels.get("Close file")))
         {
            hdf5Manager.closeFile();
            hdf5Manager = null;
            selectedFileName = "";
            openFile = "";
         }

         long numberOfObjects = imageGroup.getNumObjs();
         ImGui.text(String.format("Number of objects: %d", numberOfObjects));

         if (pngBytesPointer != null)
         {
            if (ImGui.sliderScalar(labels.get("Index"), ImGuiDataType.U32, imageIndex, 0, numberOfObjects - 1, "%d"))
            {

//               DataSet dataSet = imageGroup.openDataSet(String.valueOf(imageIndex.get()));
//
//               int size = HDF5Tools.extractShape(dataSet, 0);
//               pngBytesPointer.position(0);
//               pngBytesPointer.limit(size);
//
//               int rank = 1;
//               long[] dimensions = { pngBytesPointer.limit() };
//               DataSpace dataSpace = new DataSpace(rank, dimensions);
//               DataType dataType = new DataType(PredType.NATIVE_B8());
//               // TODO: Check if this stuff is needed
//               DSetMemXferPropList dSetMemXferPropList = new DSetMemXferPropList();
//               DataSpace fileSpace = new DataSpace(rank, dimensions);
//               dataSet.read(pngBytesPointer, dataType, dataSpace, fileSpace, dSetMemXferPropList);
//               dataSet.close();

               byte[] bytes = HDF5Tools.loadByteArray(imageGroup, (int) imageIndex.get());

               bgrImage.data().position(0);
               bgrImage.data().put(bytes);

//               pngBytesPointer.position(0);
//               pngBytesPointer.limit(bytes.length);
//               pngBytesPointer.put(bytes);
//
//               decompressionInputMat.cols((int) pngBytesPointer.limit());
//               decompressionInputMat.data(pngBytesPointer);
//
//               opencv_imgcodecs.imdecode(decompressionInputMat, opencv_imgcodecs.IMREAD_UNCHANGED, bgrImage);
               opencv_imgproc.cvtColor(bgrImage, imagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
            }
         }
      }
   }

   private void onHDF5FileSelected(String hdf5FileName)
   {
      selectedFileName = hdf5FileName;
      openFile = Paths.get(logDirectory.getDirectoryName(), hdf5FileName).toString();
      hdf5Manager = new HDF5Manager(openFile, hdf5.H5F_ACC_RDONLY);
      imageGroup = hdf5Manager.openGroup(WebcamHDF5LoggingDemo.IMAGE_GROUP_NAME);
   }

   public static void main(String[] args)
   {
      new HDF5ImageBrowserDemo();
   }
}
