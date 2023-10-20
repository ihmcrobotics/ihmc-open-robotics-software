package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXImagePanel;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVArUcoMarker;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetection;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * This class provides tuners and performance analysis for a locally running instance
 * of {@link OpenCVArUcoMarkerDetection}.
 *
 * TODO: Create a remote version.
 */
public class RDXOpenCVArUcoMarkerDetectionUI
{
   private final String namePostfix;
   private ReferenceFrame cameraFrame;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private final ArrayList<RDXOpenCVArUcoTrackedMarker> trackedMarkers = new ArrayList<>();
   private final HashMap<Integer, RDXOpenCVArUcoTrackedMarker> idToTrackedMarkerMap = new HashMap<>();
   private BytedecoImage imageForDrawing;
   private RDXMatImagePanel markerImagePanel;
   private final RDXPanel mainPanel;
   private Scalar idColor;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt adaptiveThresholdWindowSizeMin = new ImInt();
   private final ImInt adaptiveThresholdWindowSizeMax = new ImInt();
   private final ImInt adaptiveThresholdWindowSizeStep = new ImInt();
   private final ImDouble adaptiveThreshConstant = new ImDouble();
   private final ImDouble minMarkerPerimeterRate = new ImDouble();
   private final ImDouble maxMarkerPerimeterRate = new ImDouble();
   private final ImDouble polygonalApproxAccuracyRate = new ImDouble();
   private final ImDouble minCornerDistanceRate = new ImDouble();
   private final ImDouble minMarkerDistanceRate = new ImDouble();
   private final ImInt minDistanceToBorder = new ImInt();
   private final ImInt markerBorderBits = new ImInt();
   private final ImDouble minOtsuStdDev = new ImDouble();
   private final ImInt perspectiveRemovePixelPerCell = new ImInt();
   private final ImDouble perspectiveRemoveIgnoredMarginPerCell = new ImDouble();
   private final ImDouble maxErroneousBitsInBorderRate = new ImDouble();
   private final ImDouble errorCorrectionRate = new ImDouble();
   private final ImBoolean detectInvertedMarker = new ImBoolean();
   private final ImBoolean detectionEnabled = new ImBoolean(true);
   private final ImBoolean showGraphics = new ImBoolean(true);
   private ArrayList<OpenCVArUcoMarker> markersToTrack;
   private final ArrayList<RDXModelInstance> markerPoseCoordinateFrames = new ArrayList<>();
   private final FramePose3D markerPose = new FramePose3D();
   private final ImPlotPlot detectionDurationPlot = new ImPlotPlot(70);
   private final ImPlotDoublePlotLine detectionDurationPlotLine = new ImPlotDoublePlotLine("Detection duration");
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final ImPlotDoublePlotLine restOfStuffPlotLine = new ImPlotDoublePlotLine("Other stuff");

   public RDXOpenCVArUcoMarkerDetectionUI()
   {
      this("");
   }

   public RDXOpenCVArUcoMarkerDetectionUI(String namePostfix)
   {
      this.namePostfix = namePostfix;
      mainPanel = new RDXPanel("ArUco Marker Detection" + namePostfix, this::renderImGuiWidgets);
   }

   public void create(OpenCVArUcoMarkerDetection arUcoMarkerDetection)
   {
      this.arUcoMarkerDetection = arUcoMarkerDetection;

      imageForDrawing = new BytedecoImage(100, 100, opencv_core.CV_8UC3);
      boolean flipY = false;
      markerImagePanel = new RDXMatImagePanel("ArUco Marker Detection Image" + namePostfix, 100, 100, flipY);
      mainPanel.addChild(markerImagePanel.getImagePanel());

      adaptiveThresholdWindowSizeMin.set(arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeMin());
      adaptiveThresholdWindowSizeMax.set(arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeMax());
      adaptiveThresholdWindowSizeStep.set(arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeStep());
      adaptiveThreshConstant.set(arUcoMarkerDetection.getDetectorParameters().adaptiveThreshConstant());
      minMarkerPerimeterRate.set(arUcoMarkerDetection.getDetectorParameters().minMarkerPerimeterRate());
      maxMarkerPerimeterRate.set(arUcoMarkerDetection.getDetectorParameters().maxMarkerPerimeterRate());
      polygonalApproxAccuracyRate.set(arUcoMarkerDetection.getDetectorParameters().polygonalApproxAccuracyRate());
      minCornerDistanceRate.set(arUcoMarkerDetection.getDetectorParameters().minCornerDistanceRate());
      minMarkerDistanceRate.set(arUcoMarkerDetection.getDetectorParameters().minMarkerDistanceRate());
      minDistanceToBorder.set(arUcoMarkerDetection.getDetectorParameters().minDistanceToBorder());
      markerBorderBits.set(arUcoMarkerDetection.getDetectorParameters().markerBorderBits());
      minOtsuStdDev.set(arUcoMarkerDetection.getDetectorParameters().minOtsuStdDev());
      perspectiveRemovePixelPerCell.set(arUcoMarkerDetection.getDetectorParameters().perspectiveRemovePixelPerCell());
      perspectiveRemoveIgnoredMarginPerCell.set(arUcoMarkerDetection.getDetectorParameters().perspectiveRemoveIgnoredMarginPerCell());
      maxErroneousBitsInBorderRate.set(arUcoMarkerDetection.getDetectorParameters().maxErroneousBitsInBorderRate());
      errorCorrectionRate.set(arUcoMarkerDetection.getDetectorParameters().errorCorrectionRate());
      detectInvertedMarker.set(arUcoMarkerDetection.getDetectorParameters().detectInvertedMarker());

      idColor = new Scalar(0, 0, 255, 0);

      detectionDurationPlot.getPlotLines().add(detectionDurationPlotLine);
      detectionDurationPlot.getPlotLines().add(restOfStuffPlotLine);
   }

   /**
    * Rendering the 3D poses of the ArUco markers requires knowing the sizes of them,
    * which we don't always know, so we make this a separate method so it can be added
    * if needed. We might not have the camera frame either, and it's also required,
    * so we pass that in here too.
    */
   public void setupForRenderingDetectedPosesIn3D(ArrayList<OpenCVArUcoMarker> markersToTrack, ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;
      this.markersToTrack = markersToTrack;

      for (OpenCVArUcoMarker markerToTrack : markersToTrack)
      {
         RDXModelInstance coordinateFrame = new RDXModelInstance(RDXModelBuilder.createCoordinateFrame(0.4));
         markerPoseCoordinateFrames.add(coordinateFrame);
      }
   }

   public void update()
   {
      synchronized (arUcoMarkerDetection.getSyncObject())
      {
         if (detectionEnabled.get() && arUcoMarkerDetection.getHasDetected())
         {
            stopwatch.lap();
            detectionDurationPlotLine.addValue(arUcoMarkerDetection.getTimeTakenToDetect());

            if (markerImagePanel.getImagePanel().getIsShowing().get())
            {
               arUcoMarkerDetection.getCopyOfSourceRGBImage(imageForDrawing);

               arUcoMarkerDetection.drawDetectedMarkers(imageForDrawing.getBytedecoOpenCVMat(), idColor);
               arUcoMarkerDetection.drawRejectedPoints(imageForDrawing.getBytedecoOpenCVMat());

               markerImagePanel.ensureDimensionsMatch(imageForDrawing.getImageWidth(), imageForDrawing.getImageHeight());
               opencv_imgproc.cvtColor(imageForDrawing.getBytedecoOpenCVMat(), markerImagePanel.getImage(), opencv_imgproc.COLOR_RGB2RGBA);

               markerImagePanel.display();
            }

            if (markersToTrack != null && showGraphics.get())
            {
               for (int i = 0; i < markersToTrack.size(); i++)
               {
                  OpenCVArUcoMarker markerToTrack = markersToTrack.get(i);
                  if (arUcoMarkerDetection.isDetected(markerToTrack.getId()))
                  {
                     markerPose.setToZero(cameraFrame);
                     arUcoMarkerDetection.getPose(markerToTrack.getId(), markerToTrack.getSideLength(), markerPose);
                     markerPose.changeFrame(ReferenceFrame.getWorldFrame());
                     markerPoseCoordinateFrames.get(i).setPoseInWorldFrame(markerPose);
                  }
               }
            }
            restOfStuffPlotLine.addValue(stopwatch.lapElapsed());
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Detection enabled"), detectionEnabled))
         arUcoMarkerDetection.setEnabled(detectionEnabled.get());
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show 3D graphics"), showGraphics);
      ImGui.text("Image width: " + imageForDrawing.getImageWidth() + " height: " + imageForDrawing.getImageHeight());
      detectionDurationPlot.render();
      ImGui.text("Detected ArUco Markers:");
      for (RDXOpenCVArUcoTrackedMarker trackedMarker : trackedMarkers)
      {
         trackedMarker.setCurrentlyDetected(false);
      }
      synchronized (arUcoMarkerDetection.getSyncObject())
      {
         for (int i = 0; i < arUcoMarkerDetection.getDetectedIDs().size(); i++)
         {
            int id = arUcoMarkerDetection.getDetectedIDs().get(i);
            RDXOpenCVArUcoTrackedMarker trackedMarker = idToTrackedMarkerMap.get(id);
            if (trackedMarker == null)
            {
               trackedMarker = new RDXOpenCVArUcoTrackedMarker(id);
               idToTrackedMarkerMap.put(id, trackedMarker);
               trackedMarkers.add(trackedMarker);
            }
            trackedMarker.setCurrentlyDetected(true);
         }
      }
      for (RDXOpenCVArUcoTrackedMarker trackedMarker : trackedMarkers)
      {
         ImGui.text("ID: " + trackedMarker.getId());
         ImGui.sameLine();
         trackedMarker.renderPlotLine();
      }

      ImGui.text("Rejected image points: " + arUcoMarkerDetection.getNumberOfRejectedPoints());

      ImGui.pushItemWidth(150.0f);
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMin (3)"), adaptiveThresholdWindowSizeMin))
      {
         if (adaptiveThresholdWindowSizeMin.get() < 3)
            adaptiveThresholdWindowSizeMin.set(3);
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeMin(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMax (23)"), adaptiveThresholdWindowSizeMax))
      {
         if (adaptiveThresholdWindowSizeMax.get() < 3)
            adaptiveThresholdWindowSizeMax.set(3);
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeMax(adaptiveThresholdWindowSizeMax.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeStep (10)"), adaptiveThresholdWindowSizeStep))
      {
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeStep(adaptiveThresholdWindowSizeStep.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("adaptiveThreshConstant (7)"), adaptiveThreshConstant, 1.0, 30.0))
      {
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshConstant(adaptiveThreshConstant.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minMarkerPerimeterRate (0.03)"), minMarkerPerimeterRate, 0.0001, 0.1))
      {
         if (minMarkerPerimeterRate.get() <= 0)
            minMarkerPerimeterRate.set(0.0001);
         arUcoMarkerDetection.getDetectorParameters().minMarkerPerimeterRate(minMarkerPerimeterRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("maxMarkerPerimeterRate (4.0)"), maxMarkerPerimeterRate, 1.0, 10.0))
      {
         if (maxMarkerPerimeterRate.get() <= 0)
            maxMarkerPerimeterRate.set(0.0001);
         arUcoMarkerDetection.getDetectorParameters().maxMarkerPerimeterRate(maxMarkerPerimeterRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("polygonalApproxAccuracyRate (0.03)"), polygonalApproxAccuracyRate, 0.0001, 0.1))
      {
         if (polygonalApproxAccuracyRate.get() <= 0)
            polygonalApproxAccuracyRate.set(0.0001);
         arUcoMarkerDetection.getDetectorParameters().polygonalApproxAccuracyRate(polygonalApproxAccuracyRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minCornerDistanceRate (0.05)"), minCornerDistanceRate, 0.001, 0.2))
      {
         arUcoMarkerDetection.getDetectorParameters().minCornerDistanceRate(minCornerDistanceRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minMarkerDistanceRate (0.05)"), minMarkerDistanceRate, 0.001, 0.2))
      {
         arUcoMarkerDetection.getDetectorParameters().minMarkerDistanceRate(minMarkerDistanceRate.get());
      }
      if (ImGui.inputInt(labels.get("minDistanceToBorder (3px)"), minDistanceToBorder))
      {
         arUcoMarkerDetection.getDetectorParameters().minDistanceToBorder(minDistanceToBorder.get());
      }
      if (ImGui.inputInt(labels.get("markerBorderBits (1)"), markerBorderBits))
      {
         arUcoMarkerDetection.getDetectorParameters().markerBorderBits(markerBorderBits.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minOtsuStdDev (5.0)"), minOtsuStdDev, 0.5, 10.0))
      {
         arUcoMarkerDetection.getDetectorParameters().minOtsuStdDev(minOtsuStdDev.get());
      }
      if (ImGui.inputInt(labels.get("perspectiveRemovePixelPerCell (4)"), perspectiveRemovePixelPerCell))
      {
         arUcoMarkerDetection.getDetectorParameters().perspectiveRemovePixelPerCell(perspectiveRemovePixelPerCell.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("perspectiveRemoveIgnoredMarginPerCell (0.13)"), perspectiveRemoveIgnoredMarginPerCell, 0.01, 0.5))
      {
         arUcoMarkerDetection.getDetectorParameters().perspectiveRemoveIgnoredMarginPerCell(perspectiveRemoveIgnoredMarginPerCell.get());
      }
      if (ImGui.inputDouble(labels.get("maxErroneousBitsInBorderRate (0.35)"), maxErroneousBitsInBorderRate))
      {
         arUcoMarkerDetection.getDetectorParameters().maxErroneousBitsInBorderRate(maxErroneousBitsInBorderRate.get());
      }
      if (ImGui.inputDouble(labels.get("errorCorrectionRate (0.6)"), errorCorrectionRate))
      {
         arUcoMarkerDetection.getDetectorParameters().errorCorrectionRate(errorCorrectionRate.get());
      }
      if (ImGui.checkbox(labels.get("detectInvertedMarker (false)"), detectInvertedMarker))
      {
         arUcoMarkerDetection.getDetectorParameters().detectInvertedMarker(detectInvertedMarker.get());
      }
      ImGui.popItemWidth();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGraphics.get())
      {
         for (RDXModelInstance markerPoseCoordinateFrame : markerPoseCoordinateFrames)
         {
            markerPoseCoordinateFrame.getRenderables(renderables, pool);
         }
      }
   }

   public RDXPanel getMainPanel()
   {
      return mainPanel;
   }

   public RDXImagePanel getMarkerImagePanel()
   {
      return markerImagePanel.getImagePanel();
   }
}
