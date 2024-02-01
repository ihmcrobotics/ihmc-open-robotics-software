package us.ihmc.rdx.perception;

import georegression.struct.point.Vector2D_F32;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_video;
import org.bytedeco.opencv.opencv_core.*;

import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.perception.BytedecoImage;

public class RDXOpenCVOpticalFlowTrackingUI
{
   private int imageWidth;
   private int imageHeight;
   private BytedecoImage sourceImage;
   private RDXBytedecoImagePanel trackingImagePanel;
   private final RDXPanel mainPanel;

   private final Point2f previousPoint2f = new Point2f(0, 0);
   private final Point2f nextPoint2f = new Point2f(0, 0);
   private final Point firstNextPointAsInt = new Point(0, 0);
   private final Mat previousPoints = new Mat(previousPoint2f);
   private final Mat nextPoints = new Mat(nextPoint2f);
   private Mat previousImage = null;
   private Mat nextImage = null;
   private final Mat status = new Mat();
   private final Mat error = new Mat();
   private final Vector2D_F32 firstNextPoint = new Vector2D_F32(0, 0);
   private final Scalar RED = new Scalar(255, 1, 2, 255);
   private final Scalar BLUE = new Scalar(1, 1, 255, 255);
   private final TermCriteria terminationCriteria = new TermCriteria(opencv_core.CV_TERMCRIT_EPS | opencv_core.CV_TERMCRIT_NUMBER, 30, 1);
   private final Size searchWindowSize = new Size(12, 12);
   private float mousePositionInImagePixelCoordinatesX;
   private float mousePositionInImagePixelCoordinatesY;
   private int maxDistance = 90;
   private boolean userClicked = true;

   public RDXOpenCVOpticalFlowTrackingUI()
   {
      mainPanel = new RDXPanel("Door Handle Detection", this::renderImGuiWidgets);
   }

   public void create(BytedecoImage sourceImage)
   {
      this.sourceImage = sourceImage;
      imageWidth = sourceImage.getImageWidth();
      imageHeight = sourceImage.getImageHeight();
      boolean flipY = false;
      trackingImagePanel = new RDXBytedecoImagePanel("Door Handle Detection Image", imageWidth, imageHeight, flipY);
      trackingImagePanel.getImagePanel().setUserImGuiImageInteraction(this::videoPanelImGuiImageInteraction);
      mainPanel.addChild(trackingImagePanel.getImagePanel());
   }

   private void videoPanelImGuiImageInteraction()
   {
      getCorrectedMouseLocation();
      if (ImGui.isMouseReleased(ImGuiMouseButton.Left))
      {
         userClicked = true;
         previousPoint2f.x(mousePositionInImagePixelCoordinatesX);
         previousPoint2f.y(mousePositionInImagePixelCoordinatesY);
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
      ImGui.text("firstNextPointAsInt x: " + firstNextPointAsInt.x() + "PreviousPoints y: " + firstNextPointAsInt.y());
   }

   public void update()
   {
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

      opencv_imgproc.cvtColor(sourceImage.getBytedecoOpenCVMat(), nextImage, opencv_imgproc.COLOR_RGBA2GRAY);

      sourceImage.getBytedecoOpenCVMat().copyTo(trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat());

      if (true) // userClicked
      {
         // More pyramids lead to lower quality
         int maximalPyramidLevelNumber = 5; // 1 means two levels
         int operationFlags = 0;
         double minEigenValueThreshold = 0.01;
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

         firstNextPoint.set(nextPoints.ptr(0).getFloat(), nextPoints.ptr().getFloat(Float.BYTES));

         // Check if the marker moved too far over the past frame
           /* double dist = Math.sqrt(((int)xy.getX() - (int)oldPoints.getX())^2 + ((int)xy.getX() - (int)oldPoints.getX())^2);
            if((int)dist>max_distance){
               p1=p0;
               xy.set(p1.ptr(0).getFloat(), p1.ptr(0).getFloat(Float.BYTES));
            }*/

         firstNextPointAsInt.x((int) firstNextPoint.getX());
         firstNextPointAsInt.y((int) firstNextPoint.getY());

         opencv_imgproc.circle(trackingImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), firstNextPointAsInt, 8, RED, -1, opencv_imgproc.LINE_8, 0);

         nextPoints.copyTo(previousPoints);
         nextImage.copyTo(previousImage);
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
      mousePositionInImagePixelCoordinatesX = trackingImagePanel.getImagePanel().getMouseXRightFromLeft();
      mousePositionInImagePixelCoordinatesY = trackingImagePanel.getImagePanel().getMouseYDownFromTop() - ImGui.getWindowSizeY() / 2;

      float ratio = imageHeight / imageWidth;
      float realRatio = ImGui.getWindowSizeY() / ImGui.getWindowSizeX();
      if (realRatio > ratio)
      {
         float adjustedHeight = imageHeight * ImGui.getWindowSizeX() / imageWidth;
         mousePositionInImagePixelCoordinatesY -= adjustedHeight / 2 + ImGui.getTextLineHeight() - adjustedHeight;
         mousePositionInImagePixelCoordinatesY *= imageWidth / ImGui.getWindowSizeX();
         mousePositionInImagePixelCoordinatesX -= 3; // I don't know how much padding there is on the sides on photo, just guessing 3
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
   }

   public RDXPanel getMainPanel()
   {
      return mainPanel;
   }
}

