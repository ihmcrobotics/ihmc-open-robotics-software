package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Gdx2DPixmap;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.imgui.ImGuiVideoPanel;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.OpenCVDoorHandleDetection;

import java.util.ArrayList;


public class GDXOpenCVDoorHandleDetectionUI
{

   private class ImGuiVideoPanelWithMouse extends ImGuiVideoPanel
   {
      float mouseX;
      float mouseY;

      public ImGuiVideoPanelWithMouse(String name, boolean flipY)
      {
         super(name, flipY);
      }

      @Override
      public void renderImGuiWidgets()
      {
         super.renderImGuiWidgets();
         if(ImGui.isMouseDown(0))
         {
            mouseX = ImGui.getMousePosX() - ImGui.getWindowPosX();
            mouseY = ImGui.getMousePosY() - ImGui.getWindowPosY() - ImGui.getWindowSizeY() / 2;
            float ratio = imageHeight / imageWidth;
            float realRatio = ImGui.getWindowSizeY() / ImGui.getWindowSizeX();
            if (realRatio > ratio)
            {
               float adjustedHeight = imageHeight * ImGui.getWindowSizeX() / imageWidth;
               mouseY -= adjustedHeight / 2 + ImGui.getTextLineHeight();
               mouseX -= 3; //I don't know how much padding there is on the sides on photo, just guessing 3
            }
            else
            {
               mouseY -= ImGui.getWindowSizeY() / 2;
               float adjustedWidth = ImGui.getWindowSizeX()- imageWidth * (ImGui.getWindowSizeY()) / imageHeight;
               mouseX -= (adjustedWidth/2 + ImGui.getTextLineHeight()) ;
            }
            mouseY = -mouseY;



         }
         Point pointxy = new Point((int) mouseX, (int) mouseY);
         Scalar color = new Scalar(255, 1, 2, 100);
         //opencv_imgproc.circle(imageForDrawing.getBytedecoOpenCVMat(), pointxy, 100, color);

      }

      public float getMouseX()
      {
         return mouseX;
      }

      public float getMouseY()
      {
         return mouseY;
      }
   }

   public class GDXCVImagePanelWithMouse
   {
      private Pixmap pixmap;
      private final BytedecoImage bytedecoImage;
      private ImGuiVideoPanelWithMouse videoPanel;
      private Texture panelTexture;

      private BytedecoImage normalizedScaledImage;

      public GDXCVImagePanelWithMouse(String name, BytedecoImage bytedecoImage)
      {
         int imageWidth = bytedecoImage.getImageWidth();
         int imageHeight = bytedecoImage.getImageHeight();

         long[] nativeData = new long[4];
         nativeData[0] = bytedecoImage.getBytedecoByteBufferPointer().address();
         nativeData[1] = imageWidth;
         nativeData[2] = imageHeight;
         nativeData[3] = Gdx2DPixmap.GDX2D_FORMAT_RGBA8888;
         pixmap = new Pixmap(new Gdx2DPixmap(bytedecoImage.getBackingDirectByteBuffer(), nativeData));

         this.bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());

         boolean flipY = false;
         setup(name, imageWidth, imageHeight, flipY);
      }

      public GDXCVImagePanelWithMouse(String name, int imageWidth, int imageHeight)
      {
         this(name, imageWidth, imageHeight, false);
      }

      public GDXCVImagePanelWithMouse(String name, int imageWidth, int imageHeight, boolean flipY)
      {
         pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
         bytedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC4, pixmap.getPixels());

         setup(name, imageWidth, imageHeight, flipY);
      }

      private void setup(String name, int imageWidth, int imageHeight, boolean flipY)
      {
         videoPanel = new ImGuiVideoPanelWithMouse(name, flipY);

         panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
         videoPanel.setTexture(panelTexture);

         normalizedScaledImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);

         BytedecoOpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
      }

      public void drawFloatImage(Mat floatImage)
      {
         if (videoPanel.getIsShowing().get())
         {
            BytedecoOpenCVTools.clampTo8BitUnsignedChar(floatImage, normalizedScaledImage.getBytedecoOpenCVMat(), 0.0, 255.0);
            BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(normalizedScaledImage.getBytedecoOpenCVMat(), bytedecoImage.getBytedecoOpenCVMat());
            draw();
         }
      }

      /**
       * Texture must be drawn to before panel will display the image.
       * This is where the image gets transferred to the GPU.
       */
      public void draw()
      {
         bytedecoImage.rewind();

         if (videoPanel.getIsShowing().get())
            panelTexture.draw(pixmap, 0, 0);
      }

      public void resize(int imageWidth, int imageHeight, OpenCLManager openCLManager)
      {
         panelTexture.dispose();
         pixmap.dispose();

         pixmap = new Pixmap(imageWidth, imageHeight, Pixmap.Format.RGBA8888);
         panelTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
         videoPanel.setTexture(panelTexture);

         bytedecoImage.resize(imageWidth, imageHeight, openCLManager, pixmap.getPixels());
         normalizedScaledImage.resize(imageWidth, imageHeight, openCLManager, null);

         BytedecoOpenCVTools.setRGBA8888ImageAlpha(bytedecoImage.getBytedecoOpenCVMat(), 255);
      }

      public ImGuiVideoPanelWithMouse getVideoPanel()
      {
         return videoPanel;
      }

      public BytedecoImage getBytedecoImage()
      {
         return bytedecoImage;
      }

      public Pixmap getPixmap()
      {
         return pixmap;
      }
   }


   private final String namePostfix;
   private int imageWidth;
   private int imageHeight;
   private ReferenceFrame cameraFrame;
   private OpenCVDoorHandleDetection DoorHandleDetection;
   private BytedecoImage imageForDrawing;
   private GDXCVImagePanelWithMouse markerImagePanel;
   private final ImGuiPanel mainPanel;
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
   //private ArrayList<OpenCVArUcoMarker> markersToTrack;
   private final ArrayList<GDXModelInstance> markerPoseCoordinateFrames = new ArrayList<>();
   private final FramePose3D markerPose = new FramePose3D();
   private final ImPlotPlot detectionDurationPlot = new ImPlotPlot(70);
   private final ImPlotDoublePlotLine detectionDurationPlotLine = new ImPlotDoublePlotLine("Detection duration");
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final ImPlotDoublePlotLine restOfStuffPlotLine = new ImPlotDoublePlotLine("Other stuff");

   public GDXOpenCVDoorHandleDetectionUI(String namePostfix)
   {
      this.namePostfix = namePostfix;
      mainPanel = new ImGuiPanel("Door Handle Detection " + namePostfix, this::renderImGuiWidgets);
   }

   public void create(OpenCVDoorHandleDetection DoorHandleDetection, /*ArrayList<OpenCVArUcoMarker> markersToTrack,*/ ReferenceFrame cameraFrame)
   {
      this.DoorHandleDetection = DoorHandleDetection;
      //this.markersToTrack = markersToTrack;
      this.cameraFrame = cameraFrame;

      imageWidth = DoorHandleDetection.getImageWidth();
      imageHeight = DoorHandleDetection.getImageHeight();
      imageForDrawing = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      boolean flipY = false;
      markerImagePanel = new GDXCVImagePanelWithMouse("Door Handle Detection Image " + namePostfix, imageWidth, imageHeight, flipY);
      mainPanel.addChild(markerImagePanel.getVideoPanel());


      adaptiveThresholdWindowSizeMin.set(DoorHandleDetection.getDetectorParameters().adaptiveThreshWinSizeMin());
      adaptiveThresholdWindowSizeMax.set(DoorHandleDetection.getDetectorParameters().adaptiveThreshWinSizeMax());
      adaptiveThresholdWindowSizeStep.set(DoorHandleDetection.getDetectorParameters().adaptiveThreshWinSizeStep());
      adaptiveThreshConstant.set(DoorHandleDetection.getDetectorParameters().adaptiveThreshConstant());
      minMarkerPerimeterRate.set(DoorHandleDetection.getDetectorParameters().minMarkerPerimeterRate());
      maxMarkerPerimeterRate.set(DoorHandleDetection.getDetectorParameters().maxMarkerPerimeterRate());
      polygonalApproxAccuracyRate.set(DoorHandleDetection.getDetectorParameters().polygonalApproxAccuracyRate());
      minCornerDistanceRate.set(DoorHandleDetection.getDetectorParameters().minCornerDistanceRate());
      minMarkerDistanceRate.set(DoorHandleDetection.getDetectorParameters().minMarkerDistanceRate());
      minDistanceToBorder.set(DoorHandleDetection.getDetectorParameters().minDistanceToBorder());
      markerBorderBits.set(DoorHandleDetection.getDetectorParameters().markerBorderBits());
      minOtsuStdDev.set(DoorHandleDetection.getDetectorParameters().minOtsuStdDev());
      perspectiveRemovePixelPerCell.set(DoorHandleDetection.getDetectorParameters().perspectiveRemovePixelPerCell());
      perspectiveRemoveIgnoredMarginPerCell.set(DoorHandleDetection.getDetectorParameters().perspectiveRemoveIgnoredMarginPerCell());
      maxErroneousBitsInBorderRate.set(DoorHandleDetection.getDetectorParameters().maxErroneousBitsInBorderRate());
      errorCorrectionRate.set(DoorHandleDetection.getDetectorParameters().errorCorrectionRate());
      detectInvertedMarker.set(DoorHandleDetection.getDetectorParameters().detectInvertedMarker());



      idColor = new Scalar(0, 0, 255, 0);
/*
      for (OpenCVArUcoMarker markerToTrack : markersToTrack)
      {
         GDXModelInstance coordinateFrame = new GDXModelInstance(GDXModelPrimitives.createCoordinateFrame(0.4));
         markerPoseCoordinateFrames.add(coordinateFrame);
      }
   */


      detectionDurationPlot.getPlotLines().add(detectionDurationPlotLine);
      detectionDurationPlot.getPlotLines().add(restOfStuffPlotLine);
   }

   public void update()
   {



      if (detectionEnabled.get())
      {
         stopwatch.lap();
         detectionDurationPlotLine.addValue(DoorHandleDetection.getTimeTakenToDetect());

         if (markerImagePanel.getVideoPanel().getIsShowing().get())
         {

            DoorHandleDetection.getImageOfDetection(imageForDrawing.getBytedecoOpenCVMat());

            DoorHandleDetection.drawDetectedMarkers(imageForDrawing.getBytedecoOpenCVMat(), idColor);
            DoorHandleDetection.drawRejectedPoints(imageForDrawing.getBytedecoOpenCVMat());

            opencv_imgproc.cvtColor(imageForDrawing.getBytedecoOpenCVMat(), markerImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);

            float mouseX = markerImagePanel.getVideoPanel().mouseX;
            float mouseY = markerImagePanel.getVideoPanel().mouseY;

            Point pointxy = new Point((int) mouseX, (int) mouseY);
            Scalar color = new Scalar(255, 1, 2, 0);
            opencv_imgproc.circle(imageForDrawing.getBytedecoOpenCVMat(), pointxy, 100, color);
            System.out.printf("X: %f, Y: %f\n", mouseX, mouseY);

            markerImagePanel.draw();



         }

         if (showGraphics.get())
         {
            /*for (int i = 0; i < markersToTrack.size(); i++)
            {
               OpenCVArUcoMarker markerToTrack = markersToTrack.get(i);
               if (DoorHandleDetection.isDetected(markerToTrack))
               {
                  markerPose.setToZero(cameraFrame);
                  DoorHandleDetection.getPose(markerToTrack, markerPose);
                  markerPose.changeFrame(ReferenceFrame.getWorldFrame());
                  markerPoseCoordinateFrames.get(i).setPoseInWorldFrame(markerPose);
               }
            }*/
         }


         restOfStuffPlotLine.addValue(stopwatch.lapElapsed());

      }
   }

   public void renderImGuiWidgets()
   {

      if (ImGui.checkbox(labels.get("Detection enabled"), detectionEnabled))
         DoorHandleDetection.setEnabled(detectionEnabled.get());
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
      ImGui.text("Image width: " + imageWidth + " height: " + imageHeight);
      detectionDurationPlot.render();
      ImGui.text("Detected Door Handle Markers:");
      DoorHandleDetection.forEachDetectedID(id -> ImGui.text("ID: " + id));
      ImGui.text("Rejected image points: " + DoorHandleDetection.getNumberOfRejectedPoints());


      ImGui.pushItemWidth(150.0f);

      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMin"), adaptiveThresholdWindowSizeMin))
      {
         if (adaptiveThresholdWindowSizeMin.get() < 3)
            adaptiveThresholdWindowSizeMin.set(3);
         DoorHandleDetection.getDetectorParameters().adaptiveThreshWinSizeMin(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeMax"), adaptiveThresholdWindowSizeMax))
      {
         if (adaptiveThresholdWindowSizeMax.get() < 3)
            adaptiveThresholdWindowSizeMax.set(3);
         DoorHandleDetection.getDetectorParameters().adaptiveThreshWinSizeMax(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputInt(labels.get("adaptiveThresholdWindowSizeStep"), adaptiveThresholdWindowSizeStep))
      {
         DoorHandleDetection.getDetectorParameters().adaptiveThreshWinSizeStep(adaptiveThresholdWindowSizeMin.get());
      }
      if (ImGui.inputDouble(labels.get("adaptiveThreshConstant"), adaptiveThreshConstant))
      {
         DoorHandleDetection.getDetectorParameters().adaptiveThreshConstant(adaptiveThreshConstant.get());
      }
      if (ImGui.inputDouble(labels.get("minMarkerPerimeterRate"), minMarkerPerimeterRate))
      {
         if (minMarkerPerimeterRate.get() <= 0)
            minMarkerPerimeterRate.set(0.0001);
         DoorHandleDetection.getDetectorParameters().minMarkerPerimeterRate(minMarkerPerimeterRate.get());
      }
      if (ImGui.inputDouble(labels.get("maxMarkerPerimeterRate"), maxMarkerPerimeterRate))
      {
         if (maxMarkerPerimeterRate.get() <= 0)
            maxMarkerPerimeterRate.set(0.0001);
         DoorHandleDetection.getDetectorParameters().maxMarkerPerimeterRate(maxMarkerPerimeterRate.get());
      }
      if (ImGui.inputDouble(labels.get("polygonalApproxAccuracyRate"), polygonalApproxAccuracyRate))
      {
         if (polygonalApproxAccuracyRate.get() <= 0)
            polygonalApproxAccuracyRate.set(0.0001);
         DoorHandleDetection.getDetectorParameters().polygonalApproxAccuracyRate(polygonalApproxAccuracyRate.get());
      }
      if (ImGui.inputDouble(labels.get("minCornerDistanceRate"), minCornerDistanceRate))
      {
         DoorHandleDetection.getDetectorParameters().minCornerDistanceRate(minCornerDistanceRate.get());
      }
      if (ImGui.inputDouble(labels.get("minMarkerDistanceRate"), minMarkerDistanceRate))
      {
         DoorHandleDetection.getDetectorParameters().minMarkerDistanceRate(minMarkerDistanceRate.get());
      }
      if (ImGui.inputInt(labels.get("minDistanceToBorder"), minDistanceToBorder))
      {
         DoorHandleDetection.getDetectorParameters().minDistanceToBorder(minDistanceToBorder.get());
      }
      if (ImGui.inputInt(labels.get("markerBorderBits"), markerBorderBits))
      {
         DoorHandleDetection.getDetectorParameters().markerBorderBits(markerBorderBits.get());
      }
      if (ImGui.inputDouble(labels.get("minOtsuStdDev"), minOtsuStdDev))
      {
         DoorHandleDetection.getDetectorParameters().minOtsuStdDev(minOtsuStdDev.get());
      }
      if (ImGui.inputInt(labels.get("perspectiveRemovePixelPerCell"), perspectiveRemovePixelPerCell))
      {
         DoorHandleDetection.getDetectorParameters().perspectiveRemovePixelPerCell(perspectiveRemovePixelPerCell.get());
      }
      if (ImGui.inputDouble(labels.get("perspectiveRemoveIgnoredMarginPerCell"), perspectiveRemoveIgnoredMarginPerCell))
      {
         DoorHandleDetection.getDetectorParameters().perspectiveRemoveIgnoredMarginPerCell(perspectiveRemoveIgnoredMarginPerCell.get());
      }
      if (ImGui.inputDouble(labels.get("maxErroneousBitsInBorderRate"), maxErroneousBitsInBorderRate))
      {
         DoorHandleDetection.getDetectorParameters().maxErroneousBitsInBorderRate(maxErroneousBitsInBorderRate.get());
      }
      if (ImGui.inputDouble(labels.get("errorCorrectionRate"), errorCorrectionRate))
      {
         DoorHandleDetection.getDetectorParameters().errorCorrectionRate(errorCorrectionRate.get());
      }
      if (ImGui.checkbox(labels.get("detectInvertedMarker"), detectInvertedMarker))
      {
         DoorHandleDetection.getDetectorParameters().detectInvertedMarker(detectInvertedMarker.get());
      }


      ImGui.popItemWidth();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

      if (showGraphics.get())
      {
         for (GDXModelInstance markerPoseCoordinateFrame : markerPoseCoordinateFrames)
         {
            markerPoseCoordinateFrame.getRenderables(renderables, pool);

         }
      }


   }

   public ImGuiPanel getMainPanel()
   {
      return mainPanel;
   }

   public GDXCVImagePanelWithMouse getMarkerImagePanel()
   {

      return markerImagePanel;

   }

}
