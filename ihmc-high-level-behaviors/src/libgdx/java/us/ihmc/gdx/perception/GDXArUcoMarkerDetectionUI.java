package us.ihmc.gdx.perception;

import imgui.internal.ImGui;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_aruco;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.perception.ArUcoMarkerDetection;
import us.ihmc.perception.BytedecoImage;

public class GDXArUcoMarkerDetectionUI
{
   private ArUcoMarkerDetection arUcoMarkerDetection;
   private BytedecoImage agbrImage;
   private IntPointer fromRGBAToABGR = new IntPointer(0, 3, 1, 2, 2, 1, 3, 0);
   private GDXCVImagePanel markerImagePanel;
   private ImGuiPanel mainPanel = new ImGuiPanel("ArUco Marker Detection", this::renderImGuiWidgets);

   public void create(ArUcoMarkerDetection arUcoMarkerDetection)
   {
      this.arUcoMarkerDetection = arUcoMarkerDetection;

      agbrImage = new BytedecoImage(arUcoMarkerDetection.getAbgr8888ColorImage().getImageWidth(),
                                    arUcoMarkerDetection.getAbgr8888ColorImage().getImageHeight(),
                                    opencv_core.CV_8UC4);
      markerImagePanel = new GDXCVImagePanel("ArUco Marker Detection Image",
                                             arUcoMarkerDetection.getAbgr8888ColorImage().getImageWidth(),
                                             arUcoMarkerDetection.getAbgr8888ColorImage().getImageHeight());
      mainPanel.addChild(markerImagePanel.getVideoPanel());
   }

   public void update()
   {
//      opencv_aruco.drawDetectedMarkers(arUcoMarkerDetection.getAlphaRemovedImage().getBytedecoOpenCVMat(), arUcoMarkerDetection.getCorners());

//      opencv_imgproc.cvtColor(arUcoMarkerDetection.getAlphaRemovedImage().getBytedecoOpenCVMat(),
//                              agbrImage.getBytedecoOpenCVMat(),
//                              opencv_imgproc.COLOR_RGB2RGBA);
//
//      opencv_core.mixChannels(agbrImage.getBytedecoOpenCVMat(), 1, markerImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), 1, fromRGBAToABGR, 4);

      opencv_imgproc.cvtColor(arUcoMarkerDetection.getAlphaRemovedImage().getBytedecoOpenCVMat(),
                              markerImagePanel.getBytedecoImage().getBytedecoOpenCVMat(),
                              opencv_imgproc.COLOR_RGB2RGBA);

//      opencv_core.mixChannels(agbrImage.getBytedecoOpenCVMat(), 1, markerImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), 1, fromRGBAToABGR, 4);

      markerImagePanel.draw();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Detected ArUco Markers:");
      for (Integer id : arUcoMarkerDetection.getIds())
      {
         ImGui.text("ID: " + id);
      }
   }

   public ImGuiPanel getMainPanel()
   {
      return mainPanel;
   }
}
