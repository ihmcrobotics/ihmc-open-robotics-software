package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetectionResults;
import us.ihmc.perception.opencv.OpenCVArUcoMarkerDetector;
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
import us.ihmc.tools.thread.SwapReference;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * This class provides tuners and performance analysis for a locally running instance
 * of {@link OpenCVArUcoMarkerDetector}.
 *
 * TODO: Create a remote version.
 */
public class RDXOpenCVArUcoMarkerDetectionUI
{
   private final String namePostfix;
   private ReferenceFrame cameraFrame;
   private final SwapReference<OpenCVArUcoMarkerDetectionResults> arUcoMarkerDetectionResultsSwapReference
                                                                = new SwapReference<>(OpenCVArUcoMarkerDetectionResults::new);
   private final ArrayList<RDXOpenCVArUcoTrackedMarker> trackedMarkers = new ArrayList<>();
   private final HashMap<Integer, RDXOpenCVArUcoTrackedMarker> idToTrackedMarkerMap = new HashMap<>();
   private RDXMatImagePanel markerImagePanel;
   private final RDXPanel mainPanel;
   private Scalar idColor;
   private DetectorParameters detectorParametersForTuners;
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
   private final RigidBodyTransform markerPoseInWorld = new RigidBodyTransform();
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

   public void create(DetectorParameters initialDetectorParameters)
   {
      boolean flipY = false;
      markerImagePanel = new RDXMatImagePanel("ArUco Marker Detection Image" + namePostfix, 100, 100, flipY);
      mainPanel.addChild(markerImagePanel.getImagePanel());

      detectorParametersForTuners = new DetectorParameters();
      adaptiveThresholdWindowSizeMin.set(initialDetectorParameters.adaptiveThreshWinSizeMin());
      adaptiveThresholdWindowSizeMax.set(initialDetectorParameters.adaptiveThreshWinSizeMax());
      adaptiveThresholdWindowSizeStep.set(initialDetectorParameters.adaptiveThreshWinSizeStep());
      adaptiveThreshConstant.set(initialDetectorParameters.adaptiveThreshConstant());
      minMarkerPerimeterRate.set(initialDetectorParameters.minMarkerPerimeterRate());
      maxMarkerPerimeterRate.set(initialDetectorParameters.maxMarkerPerimeterRate());
      polygonalApproxAccuracyRate.set(initialDetectorParameters.polygonalApproxAccuracyRate());
      minCornerDistanceRate.set(initialDetectorParameters.minCornerDistanceRate());
      minMarkerDistanceRate.set(initialDetectorParameters.minMarkerDistanceRate());
      minDistanceToBorder.set(initialDetectorParameters.minDistanceToBorder());
      markerBorderBits.set(initialDetectorParameters.markerBorderBits());
      minOtsuStdDev.set(initialDetectorParameters.minOtsuStdDev());
      perspectiveRemovePixelPerCell.set(initialDetectorParameters.perspectiveRemovePixelPerCell());
      perspectiveRemoveIgnoredMarginPerCell.set(initialDetectorParameters.perspectiveRemoveIgnoredMarginPerCell());
      maxErroneousBitsInBorderRate.set(initialDetectorParameters.maxErroneousBitsInBorderRate());
      errorCorrectionRate.set(initialDetectorParameters.errorCorrectionRate());
      detectInvertedMarker.set(initialDetectorParameters.detectInvertedMarker());

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
      synchronized (arUcoMarkerDetectionResultsSwapReference)
      {
         OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults = arUcoMarkerDetectionResultsSwapReference.getForThreadTwo();

         if (detectionEnabled.get())
         {
            stopwatch.lap();
            detectionDurationPlotLine.addValue(arUcoMarkerDetectionResults.getTimeTakenToDetect());

            if (markerImagePanel.getImagePanel().getIsShowing().get())
            {
               BytedecoImage imageForDrawing = arUcoMarkerDetectionResults.getInputImage();

               arUcoMarkerDetectionResults.drawDetectedMarkers(imageForDrawing.getBytedecoOpenCVMat(), idColor);
               arUcoMarkerDetectionResults.drawRejectedPoints(imageForDrawing.getBytedecoOpenCVMat());

               markerImagePanel.ensureDimensionsMatch(imageForDrawing.getImageWidth(), imageForDrawing.getImageHeight());
               opencv_imgproc.cvtColor(imageForDrawing.getBytedecoOpenCVMat(), markerImagePanel.getImage(), opencv_imgproc.COLOR_RGB2RGBA);

               markerImagePanel.display();
            }

            if (markersToTrack != null && showGraphics.get())
            {
               for (int i = 0; i < markersToTrack.size(); i++)
               {
                  OpenCVArUcoMarker markerToTrack = markersToTrack.get(i);
                  boolean detected = arUcoMarkerDetectionResults.getPose(markerToTrack.getId(),
                                                                         markerToTrack.getSideLength(),
                                                                         cameraFrame,
                                                                         ReferenceFrame.getWorldFrame(),
                                                                         markerPoseInWorld);
                  if (detected)
                  {
                     markerPoseCoordinateFrames.get(i).setPoseInWorldFrame(markerPoseInWorld);
                  }
               }
            }
            restOfStuffPlotLine.addValue(stopwatch.lapElapsed());
         }
      }
   }

   public void renderImGuiWidgets()
   {
      synchronized (arUcoMarkerDetectionResultsSwapReference)
      {
         OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults = arUcoMarkerDetectionResultsSwapReference.getForThreadTwo();

         ImGui.checkbox(labels.get("Detection enabled"), detectionEnabled);
         ImGui.sameLine();
         ImGui.checkbox(labels.get("Show 3D graphics"), showGraphics);
         BytedecoImage imageForDrawing = arUcoMarkerDetectionResults.getInputImage();
         ImGui.text("Image width: " + imageForDrawing.getImageWidth() + " height: " + imageForDrawing.getImageHeight());
         detectionDurationPlot.render();
         ImGui.text("Detected ArUco Markers:");
         for (RDXOpenCVArUcoTrackedMarker trackedMarker : trackedMarkers)
         {
            trackedMarker.setCurrentlyDetected(false);
         }
         for (int i = 0; i < arUcoMarkerDetectionResults.getDetectedIDs().size(); i++)
         {
            int id = arUcoMarkerDetectionResults.getDetectedIDs().get(i);
            RDXOpenCVArUcoTrackedMarker trackedMarker = idToTrackedMarkerMap.get(id);
            if (trackedMarker == null)
            {
               trackedMarker = new RDXOpenCVArUcoTrackedMarker(id);
               idToTrackedMarkerMap.put(id, trackedMarker);
               trackedMarkers.add(trackedMarker);
            }
            trackedMarker.setCurrentlyDetected(true);
         }
         for (RDXOpenCVArUcoTrackedMarker trackedMarker : trackedMarkers)
         {
            ImGui.text("ID: " + trackedMarker.getId());
            ImGui.sameLine();
            trackedMarker.renderPlotLine();
         }

         ImGui.text("Rejected image points: " + arUcoMarkerDetectionResults.getNumberOfRejectedPoints());
      }

      ImGui.pushItemWidth(150.0f);
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMin (3)"), adaptiveThresholdWindowSizeMin))
      {
         if (adaptiveThresholdWindowSizeMin.get() < 3)
            adaptiveThresholdWindowSizeMin.set(3);
         detectorParametersForTuners.adaptiveThreshWinSizeMin(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMax (23)"), adaptiveThresholdWindowSizeMax))
      {
         if (adaptiveThresholdWindowSizeMax.get() < 3)
            adaptiveThresholdWindowSizeMax.set(3);
         detectorParametersForTuners.adaptiveThreshWinSizeMax(adaptiveThresholdWindowSizeMax.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeStep (10)"), adaptiveThresholdWindowSizeStep))
      {
         detectorParametersForTuners.adaptiveThreshWinSizeStep(adaptiveThresholdWindowSizeStep.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("adaptiveThreshConstant (7)"), adaptiveThreshConstant, 1.0, 30.0))
      {
         detectorParametersForTuners.adaptiveThreshConstant(adaptiveThreshConstant.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minMarkerPerimeterRate (0.03)"), minMarkerPerimeterRate, 0.0001, 0.1))
      {
         if (minMarkerPerimeterRate.get() <= 0)
            minMarkerPerimeterRate.set(0.0001);
         detectorParametersForTuners.minMarkerPerimeterRate(minMarkerPerimeterRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("maxMarkerPerimeterRate (4.0)"), maxMarkerPerimeterRate, 1.0, 10.0))
      {
         if (maxMarkerPerimeterRate.get() <= 0)
            maxMarkerPerimeterRate.set(0.0001);
         detectorParametersForTuners.maxMarkerPerimeterRate(maxMarkerPerimeterRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("polygonalApproxAccuracyRate (0.03)"), polygonalApproxAccuracyRate, 0.0001, 0.1))
      {
         if (polygonalApproxAccuracyRate.get() <= 0)
            polygonalApproxAccuracyRate.set(0.0001);
         detectorParametersForTuners.polygonalApproxAccuracyRate(polygonalApproxAccuracyRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minCornerDistanceRate (0.05)"), minCornerDistanceRate, 0.001, 0.2))
      {
         detectorParametersForTuners.minCornerDistanceRate(minCornerDistanceRate.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minMarkerDistanceRate (0.05)"), minMarkerDistanceRate, 0.001, 0.2))
      {
         detectorParametersForTuners.minMarkerDistanceRate(minMarkerDistanceRate.get());
      }
      if (ImGui.inputInt(labels.get("minDistanceToBorder (3px)"), minDistanceToBorder))
      {
         detectorParametersForTuners.minDistanceToBorder(minDistanceToBorder.get());
      }
      if (ImGui.inputInt(labels.get("markerBorderBits (1)"), markerBorderBits))
      {
         detectorParametersForTuners.markerBorderBits(markerBorderBits.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("minOtsuStdDev (5.0)"), minOtsuStdDev, 0.5, 10.0))
      {
         detectorParametersForTuners.minOtsuStdDev(minOtsuStdDev.get());
      }
      if (ImGui.inputInt(labels.get("perspectiveRemovePixelPerCell (4)"), perspectiveRemovePixelPerCell))
      {
         detectorParametersForTuners.perspectiveRemovePixelPerCell(perspectiveRemovePixelPerCell.get());
      }
      if (ImGuiTools.sliderDouble(labels.get("perspectiveRemoveIgnoredMarginPerCell (0.13)"), perspectiveRemoveIgnoredMarginPerCell, 0.01, 0.5))
      {
         detectorParametersForTuners.perspectiveRemoveIgnoredMarginPerCell(perspectiveRemoveIgnoredMarginPerCell.get());
      }
      if (ImGui.inputDouble(labels.get("maxErroneousBitsInBorderRate (0.35)"), maxErroneousBitsInBorderRate))
      {
         detectorParametersForTuners.maxErroneousBitsInBorderRate(maxErroneousBitsInBorderRate.get());
      }
      if (ImGui.inputDouble(labels.get("errorCorrectionRate (0.6)"), errorCorrectionRate))
      {
         detectorParametersForTuners.errorCorrectionRate(errorCorrectionRate.get());
      }
      if (ImGui.checkbox(labels.get("detectInvertedMarker (false)"), detectInvertedMarker))
      {
         detectorParametersForTuners.detectInvertedMarker(detectInvertedMarker.get());
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

   public void copyOutputData(OpenCVArUcoMarkerDetector detection)
   {
      arUcoMarkerDetectionResultsSwapReference.getForThreadOne().copyOutputData(detection);
      arUcoMarkerDetectionResultsSwapReference.swap();
   }

   public ImBoolean getDetectionEnabled()
   {
      return detectionEnabled;
   }

   public DetectorParameters getTunedDetectorParameters()
   {
      return detectorParametersForTuners;
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
