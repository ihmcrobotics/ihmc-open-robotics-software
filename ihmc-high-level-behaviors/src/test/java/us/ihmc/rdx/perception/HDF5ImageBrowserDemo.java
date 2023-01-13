package us.ihmc.rdx.perception;

import boofcv.alg.sfm.DepthSparse3D;
import imgui.ImGui;
import imgui.type.ImString;
import org.bytedeco.hdf5.*;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.logging.HDF5Manager;
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
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Comparator;
import java.util.Date;
import java.util.SortedSet;
import java.util.TreeSet;

public class HDF5ImageBrowserDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "HDF5 Image Browser Demo");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiPanel panel = new ImGuiPanel("HDF5 Browsing", this::renderImGuiWidgets);
   private RDXCVImagePanel imagePanel;
   private HDF5Manager hdf5Manager = null;
   private final ImString logDirectory = new ImString(IHMCCommonPaths.LOGS_DIRECTORY.toString());
   private boolean indexedLogFolderOnce = false;
   private final Comparator<Path> naturalOrderComparator = Comparator.comparing(path -> path.getFileName().toString());
   private final SortedSet<Path> sortedLogFolderPaths = new TreeSet<>(naturalOrderComparator.reversed());

   public HDF5ImageBrowserDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(panel);
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
         ImGuiTools.inputText(labels.get("Log directory"), logDirectory);

         boolean reindexClicked = ImGui.button(labels.get("Reindex log folder"));
         if (!indexedLogFolderOnce || reindexClicked)
         {
            indexedLogFolderOnce = true;
            reindexLogFolder();
         }
         ImGui.sameLine();
         ImGui.text("Available logs:");

         if (ImGui.button(labels.get("Begin")))
         {
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
            String logFileName = dateFormat.format(new Date()) + "_" + "Webcam.hdf5";
            FileTools.ensureDirectoryExists(Paths.get(logDirectory.get()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
            logFile = logDirectory.get() + File.separator + logFileName;
            hdf5Manager = new HDF5Manager(logFile, hdf5.H5F_ACC_TRUNC);

            imageGroup = hdf5Manager.getGroup("image");
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
               opencv_imgcodecs.imencode(".png", bgrWebcamCopy, pngImageBuffer, compressionParameters);
            }

            int rank = 1;
            long[] dimensions = { pngImageBuffer.limit() };
            DataSpace dataSpace = new DataSpace(rank, dimensions);
            DataType dataType = new DataType(PredType.NATIVE_B8());
            DataSet dataSet = imageGroup.createDataSet(String.valueOf(imageIndex), dataType, dataSpace);
            dataSet.write(pngImageBuffer, dataType);
            dataSet.close();
            dataSpace.close();

            ++imageIndex;
         }
      }

   }

   public static void main(String[] args)
   {
      new HDF5ImageBrowserDemo();
   }
}
