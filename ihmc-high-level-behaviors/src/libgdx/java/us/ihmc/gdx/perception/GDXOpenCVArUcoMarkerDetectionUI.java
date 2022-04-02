package us.ihmc.gdx.perception;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;

public class GDXOpenCVArUcoMarkerDetectionUI
{
   private final String namePostfix;
   private int imageWidth;
   private int imageHeight;
   private OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private BytedecoImage imageForDrawing;
   private GDXCVImagePanel markerImagePanel;
   private final ImGuiPanel mainPanel;
   private Rect rectangle;
   private Scalar rectangleColor;
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

   public GDXOpenCVArUcoMarkerDetectionUI(String namePostfix)
   {
      this.namePostfix = namePostfix;
      mainPanel = new ImGuiPanel("ArUco Marker Detection " + namePostfix, this::renderImGuiWidgets);
   }

   public void create(OpenCVArUcoMarkerDetection arUcoMarkerDetection)
   {
      this.arUcoMarkerDetection = arUcoMarkerDetection;

      imageWidth = arUcoMarkerDetection.getImageOfDetection().getImageWidth();
      imageHeight = arUcoMarkerDetection.getImageOfDetection().getImageHeight();
      imageForDrawing = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      boolean flipY = false;
      markerImagePanel = new GDXCVImagePanel("ArUco Marker Detection Image " + namePostfix, imageWidth, imageHeight, flipY);
      mainPanel.addChild(markerImagePanel.getVideoPanel());

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

      rectangle = new Rect();
      rectangleColor = new Scalar(255, 0, 0, 0);
      idColor = new Scalar(0, 0, 255, 0);
   }

   public void update()
   {
      arUcoMarkerDetection.getImageOfDetection().getBytedecoOpenCVMat().copyTo(imageForDrawing.getBytedecoOpenCVMat());

      opencv_aruco.drawDetectedMarkers(imageForDrawing.getBytedecoOpenCVMat(),
                                       arUcoMarkerDetection.getCorners(),
                                       arUcoMarkerDetection.getIdsAsMat(),
                                       idColor);
      opencv_aruco.drawDetectedMarkers(imageForDrawing.getBytedecoOpenCVMat(), arUcoMarkerDetection.getRejectedImagePoints());

//      for (int i = 0; i < arUcoMarkerDetection.getRejectedImagePoints().size(); i++)
//      {
//         int x = arUcoMarkerDetection.getRejectedImagePoints().get(i).ptr(0).getInt();
//         int y = arUcoMarkerDetection.getRejectedImagePoints().get(i).ptr(1).getInt();
//         rectangle.x(x);
//         rectangle.y(y);
//         opencv_imgproc.rectangle(imageForDrawing.getBytedecoOpenCVMat(), rectangle, rectangleColor);
//      }

      opencv_imgproc.cvtColor(imageForDrawing.getBytedecoOpenCVMat(),
                              markerImagePanel.getBytedecoImage().getBytedecoOpenCVMat(),
                              opencv_imgproc.COLOR_RGB2RGBA);

      markerImagePanel.draw();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Image width: " + imageWidth + " height: " + imageHeight);
      ImGui.text("Detected ArUco Markers:");
      for (Integer id : arUcoMarkerDetection.getIds())
      {
         ImGui.text("ID: " + id);
      }
      ImGui.text("Rejected image points: " + arUcoMarkerDetection.getRejectedImagePoints().size());

      ImGui.pushItemWidth(150.0f);
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMin"), adaptiveThresholdWindowSizeMin))
      {
         if (adaptiveThresholdWindowSizeMin.get() < 3)
            adaptiveThresholdWindowSizeMin.set(3);
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeMin(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMax"), adaptiveThresholdWindowSizeMax))
      {
         if (adaptiveThresholdWindowSizeMax.get() < 3)
            adaptiveThresholdWindowSizeMax.set(3);
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeMax(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeStep"), adaptiveThresholdWindowSizeStep))
      {
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshWinSizeStep(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputDouble(labels.get("adaptiveThreshConstant"), adaptiveThreshConstant))
      {
         arUcoMarkerDetection.getDetectorParameters().adaptiveThreshConstant(adaptiveThreshConstant.get());
      }
      if (ImGui.inputDouble(labels.get("minMarkerPerimeterRate"), minMarkerPerimeterRate))
      {
         if (minMarkerPerimeterRate.get() <= 0)
            minMarkerPerimeterRate.set(0.0001);
         arUcoMarkerDetection.getDetectorParameters().minMarkerPerimeterRate(minMarkerPerimeterRate.get());
      }
      if (ImGui.inputDouble(labels.get("maxMarkerPerimeterRate"), maxMarkerPerimeterRate))
      {
         if (maxMarkerPerimeterRate.get() <= 0)
            maxMarkerPerimeterRate.set(0.0001);
         arUcoMarkerDetection.getDetectorParameters().maxMarkerPerimeterRate(maxMarkerPerimeterRate.get());
      }
      if (ImGui.inputDouble(labels.get("polygonalApproxAccuracyRate"), polygonalApproxAccuracyRate))
      {
         if (polygonalApproxAccuracyRate.get() <= 0)
            polygonalApproxAccuracyRate.set(0.0001);
         arUcoMarkerDetection.getDetectorParameters().polygonalApproxAccuracyRate(polygonalApproxAccuracyRate.get());
      }
      if (ImGui.inputDouble(labels.get("minCornerDistanceRate"), minCornerDistanceRate))
      {
         arUcoMarkerDetection.getDetectorParameters().minCornerDistanceRate(minCornerDistanceRate.get());
      }
      if (ImGui.inputDouble(labels.get("minMarkerDistanceRate"), minMarkerDistanceRate))
      {
         arUcoMarkerDetection.getDetectorParameters().minMarkerDistanceRate(minMarkerDistanceRate.get());
      }
      if (ImGui.inputInt(labels.get("minDistanceToBorder"), minDistanceToBorder))
      {
         arUcoMarkerDetection.getDetectorParameters().minDistanceToBorder(minDistanceToBorder.get());
      }
      if (ImGui.inputInt(labels.get("markerBorderBits"), markerBorderBits))
      {
         arUcoMarkerDetection.getDetectorParameters().markerBorderBits(markerBorderBits.get());
      }
      if (ImGui.inputDouble(labels.get("minOtsuStdDev"), minOtsuStdDev))
      {
         arUcoMarkerDetection.getDetectorParameters().minOtsuStdDev(minOtsuStdDev.get());
      }
      if (ImGui.inputInt(labels.get("perspectiveRemovePixelPerCell"), perspectiveRemovePixelPerCell))
      {
         arUcoMarkerDetection.getDetectorParameters().perspectiveRemovePixelPerCell(perspectiveRemovePixelPerCell.get());
      }
      if (ImGui.inputDouble(labels.get("perspectiveRemoveIgnoredMarginPerCell"), perspectiveRemoveIgnoredMarginPerCell))
      {
         arUcoMarkerDetection.getDetectorParameters().perspectiveRemoveIgnoredMarginPerCell(perspectiveRemoveIgnoredMarginPerCell.get());
      }
      if (ImGui.inputDouble(labels.get("maxErroneousBitsInBorderRate"), maxErroneousBitsInBorderRate))
      {
         arUcoMarkerDetection.getDetectorParameters().maxErroneousBitsInBorderRate(maxErroneousBitsInBorderRate.get());
      }
      if (ImGui.inputDouble(labels.get("errorCorrectionRate"), errorCorrectionRate))
      {
         arUcoMarkerDetection.getDetectorParameters().errorCorrectionRate(errorCorrectionRate.get());
      }
      if (ImGui.checkbox(labels.get("detectInvertedMarker"), detectInvertedMarker))
      {
         arUcoMarkerDetection.getDetectorParameters().detectInvertedMarker(detectInvertedMarker.get());
      }
      ImGui.popItemWidth();
   }

   public ImGuiPanel getMainPanel()
   {
      return mainPanel;
   }

   public GDXCVImagePanel getMarkerImagePanel()
   {
      return markerImagePanel;
   }
}
