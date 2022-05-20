package us.ihmc.gdx.perception;

import georegression.struct.point.Vector2D_F32;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_video;
import org.bytedeco.opencv.opencv_core.*;

import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

public class GDXOpenCVDoorHandleDetectionUI
{
   private int imageWidth;
   private int imageHeight;
   private BytedecoImage sourceImage;
   private BytedecoImage imageForDrawing;
   private GDXCVImagePanel trackingImagePanel;
   private final ImGuiPanel mainPanel;
   private Scalar idColor;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   //private Scalar4f coords0 = new Scalar4f(0.0f,0.0f);
   //private Pointer point1 = new Pointer();
   //private Pointer point2 = new Pointer();
   /*MatOfPoint p0MatofPoint = new MatOfPoint();
   MatOfPoint2f p0 = new MatOfPoint2f(p0MatofPoint.toArray()) ;
   MatOfPoint2f p1 = new MatOfPoint2f(); */
   private Point2f point2f1 = new Point2f(0, 0);
   private Point2f point2f2 = new Point2f(0, 0);
   private Point firstNextPointAsInt = new Point(0, 0);
   private Point pointxy = new Point(0, 0);
   private Mat previousPoints = new Mat(point2f1);
   private Mat nextPoints = new Mat(point2f2);
   private Mat previousImage;
   private Mat nextImage;
   private Mat status = new Mat();
   private Mat error = new Mat();
   private Vector2D_F32 oldPoints = new Vector2D_F32();
   private Vector2D_F32 firstNextPoint = new Vector2D_F32();
   private int max_distance = 90;
   private Scalar RED = new Scalar(255, 1, 2, 255);
   private Scalar BLUE = new Scalar(1, 1, 255, 255);
   private TermCriteria terminationCriteria = new TermCriteria(opencv_core.CV_TERMCRIT_EPS | opencv_core.CV_TERMCRIT_NUMBER, 10, .01);
   private Size searchWindowSize = new Size(12, 12);
   private float mousePositionInImagePixelCoordinatesX;
   private float mousePositionInImagePixelCoordinatesY;
   private boolean userClicked = true;

   public GDXOpenCVDoorHandleDetectionUI()
   {
      mainPanel = new ImGuiPanel("Door Handle Detection", this::renderImGuiWidgets);
   }

   public void create(BytedecoImage sourceImage)
   {
      this.sourceImage = sourceImage;
      imageWidth = sourceImage.getImageWidth();
      imageHeight = sourceImage.getImageHeight();
      imageForDrawing = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      boolean flipY = false;
      trackingImagePanel = new GDXCVImagePanel("Door Handle Detection Image", imageWidth, imageHeight, flipY);
      trackingImagePanel.getVideoPanel().setUserImGuiImageInteraction(this::videoPanelImGuiImageInteraction);
      mainPanel.addChild(trackingImagePanel.getVideoPanel());
   }

   private void videoPanelImGuiImageInteraction()
   {
      getCorrectedMouseLocation();
      if (ImGui.isMouseReleased(ImGuiMouseButton.Left))
      {
         userClicked = true;
         LogTools.info("User left clicked. x: {} y: {}", mousePositionInImagePixelCoordinatesX, mousePositionInImagePixelCoordinatesY);
         previousPoints.ptr(0).putFloat(mousePositionInImagePixelCoordinatesX);
         previousPoints.ptr(1).putFloat(mousePositionInImagePixelCoordinatesY);
      }
      else if (ImGui.isMouseReleased(ImGuiMouseButton.Right))
      {
//         userClicked = false;
//         LogTools.info("User right clicked.");
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Image width: " + imageWidth + " height: " + imageHeight);
      ImGui.text("firstNextPointAsInt x: " + firstNextPointAsInt.x() + " y: " + firstNextPointAsInt.y());
   }

   public void update()
   {
      //      opencv_imgproc.cvtColor(imageForDrawing.getBytedecoOpenCVMat(),
      //                              trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat(),
      //                              opencv_imgproc.COLOR_RGB2RGBA);

      //      pointxy.x((int) mouseClickPositionX);
      //      pointxy.y((int) mouseClickPositionY);
      // opencv_imgproc.circle(markerImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), pointxy, 6, new Scalar(0,0,255,255), -1, opencv_imgproc.LINE_8, 0);

      if (previousImage == null)
      {
         previousImage = new Mat(imageHeight, imageWidth, sourceImage.getBytedecoOpenCVMat().type());
         opencv_imgproc.cvtColor(sourceImage.getBytedecoOpenCVMat(), previousImage, opencv_imgproc.COLOR_RGBA2GRAY);
         return;
      }

      if (nextImage == null)
      {
         nextImage = new Mat(imageHeight, imageWidth, sourceImage.getBytedecoOpenCVMat().type());
      }

      opencv_imgproc.cvtColor(sourceImage.getBytedecoOpenCVMat(), nextImage, opencv_imgproc.COLOR_RGB2GRAY);

      if (userClicked)
      {
         //More pyramids lead to lower quality
         int maximalPyramidLevelNumber = 1; // 1 means two levels
         int operationFlags = 0;
         double minEigenValueThreshold = 0.1;
         opencv_video.calcOpticalFlowPyrLK(previousImage,
                                           nextImage,
                                           previousPoints,
                                           nextPoints,
                                           status,
                                           error,
                                           searchWindowSize,
                                           maximalPyramidLevelNumber,
                                           terminationCriteria,
                                           operationFlags,
                                           minEigenValueThreshold);

         firstNextPoint.set(nextPoints.ptr(0).getFloat(), nextPoints.ptr(1).getFloat());

         //Check if the marker moved too far over the past frame
           /* double dist = Math.sqrt(((int)xy.getX() - (int)oldPoints.getX())^2 + ((int)xy.getX() - (int)oldPoints.getX())^2);
            if((int)dist>max_distance){
               p1=p0;
               xy.set(p1.ptr(0).getFloat(), p1.ptr(0).getFloat(Float.BYTES));
            }*/

         firstNextPointAsInt.x((int) firstNextPoint.getX());
         firstNextPointAsInt.y((int) firstNextPoint.getY());
         // System.out.print("X:" + xy.getX() + ", Y:" + xy.getY() + "\n");
         opencv_imgproc.circle(trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), firstNextPointAsInt, 8, RED, -1, opencv_imgproc.LINE_8, 0);
         opencv_imgproc.circle(trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat(),
                               new Point((int) mousePositionInImagePixelCoordinatesX, (int) mousePositionInImagePixelCoordinatesY),
                               4,
                               new Scalar(0, 0, 255, 255),
                               -1,
                               opencv_imgproc.LINE_8,
                               0);
//         System.out.print(firstNextPointAsInt.x() + " " + firstNextPointAsInt.y() + "\n");

         previousPoints = nextPoints;
         oldPoints.set(firstNextPoint.getX(), firstNextPoint.getY());

         nextImage.copyTo(previousImage);

         opencv_imgproc.cvtColor(nextImage, trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_RGB2RGBA);
      }
      else
      {
         sourceImage.getBytedecoOpenCVMat().copyTo(trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat());
      }

      Point currentMouseLocation = new Point((int) mousePositionInImagePixelCoordinatesX, (int) mousePositionInImagePixelCoordinatesY);
      opencv_imgproc.circle(trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), currentMouseLocation, 6, BLUE, -1, opencv_imgproc.LINE_8, 0);

      trackingImagePanel.draw();
   }

   private void getCorrectedMouseLocation()
   {
      mousePositionInImagePixelCoordinatesX = trackingImagePanel.getVideoPanel().getMouseXRightFromLeft();
      mousePositionInImagePixelCoordinatesY = trackingImagePanel.getVideoPanel().getMouseYDownFromTop() - ImGui.getWindowSizeY() / 2;
      float ratio = imageHeight / imageWidth;
      float realRatio = ImGui.getWindowSizeY() / ImGui.getWindowSizeX();
      if (realRatio > ratio)
      {
         float adjustedHeight = imageHeight * ImGui.getWindowSizeX() / imageWidth;
         mousePositionInImagePixelCoordinatesY -= adjustedHeight / 2 + ImGui.getTextLineHeight() - adjustedHeight;
         mousePositionInImagePixelCoordinatesY *= imageWidth / ImGui.getWindowSizeX();
         mousePositionInImagePixelCoordinatesX -= 3; //I don't know how much padding there is on the sides on photo, just guessing 3
         mousePositionInImagePixelCoordinatesX *= imageWidth / ImGui.getWindowSizeX();
      }
      else
      {
         mousePositionInImagePixelCoordinatesY += (ImGui.getWindowSizeY()) / 2;
         mousePositionInImagePixelCoordinatesY -= 2 * ImGui.getTextLineHeight();
         float adjustedWidth = ImGui.getWindowSizeX() - imageWidth * (ImGui.getWindowSizeY() - ImGui.getTextLineHeight()) / imageHeight;
         mousePositionInImagePixelCoordinatesX -= (adjustedWidth / 2);
         mousePositionInImagePixelCoordinatesX *= imageHeight / (ImGui.getWindowSizeY() - ImGui.getTextLineHeight());
         mousePositionInImagePixelCoordinatesY *= imageHeight / (ImGui.getWindowSizeY() - 2 * ImGui.getTextLineHeight());
      }
      //mouseY = -mouseY;
      //mouseY+= (ImGui.getWindowSizeY());

      Point pointxy = new Point((int) mousePositionInImagePixelCoordinatesX, (int) mousePositionInImagePixelCoordinatesY);
      Scalar color = new Scalar(255, 1, 2, 255);
   }

   public ImGuiPanel getMainPanel()
   {
      return mainPanel;
   }

   public GDXCVImagePanel getTrackingImagePanel()
   {
      return trackingImagePanel;
   }
}

