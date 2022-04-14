package us.ihmc.perception.realsense;

import java.nio.ShortBuffer;
import java.util.ArrayList;

import org.bytedeco.librealsense2.rs2_intrinsics;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Debug class for creating a lot of YoGraphicPositions to visualize a point cloud. 
 */
public class L515Visualizer
{
   private final ArrayList<YoGraphicPosition> positions = new ArrayList<YoGraphicPosition>();
   private int depthWidth;
   private int depthHeight;
   private int vizWidth;
   private int vizHeight;
   private ReferenceFrame sensorFrame;
   
   public L515Visualizer(String prefix, int depthWidth, int depthHeight, int vizWidth, int vizHeight, ReferenceFrame sensorFrame, YoRegistry registry,
                         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.vizWidth = vizWidth;
      this.vizHeight = vizHeight;
      this.sensorFrame = sensorFrame;
      
      for(int x = 0; x < vizWidth; x++)
      {
         for(int y = 0; y < vizHeight; y++)
         {
            YoGraphicPosition gp = new YoGraphicPosition(prefix + x + "x" + y + "y", "", registry, 0.005, YoAppearance.Aqua());
            graphicsListRegistry.registerYoGraphic("l515DepthPoints", gp);
            positions.add(gp);
         }
      }
   }
   
   float[] outPoint = new float[3];
   float[] inPixel = new float[2];
   FramePoint3D point = new FramePoint3D();
   
   public void update(ShortBuffer depthData, double depthToMeterConversion, rs2_intrinsics intrinsic_parameters)
   {
      int xStep = depthWidth / vizWidth;
      int yStep = depthHeight / vizHeight;
      
      int gpIndex = 0;
      for(int y = 0; y < vizHeight; y++)
      {
         for(int x = 0; x < vizWidth; x++)
         {
            int index = y * depthWidth * yStep  + x * xStep;
            int val =  Short.toUnsignedInt(depthData.get(index));
            double z = val * depthToMeterConversion;
            
            YoGraphicPosition gp = positions.get(gpIndex);
            
            inPixel[0] = x * xStep;
            inPixel[1] = y * yStep;
            
            realsense2.rs2_deproject_pixel_to_point(outPoint, intrinsic_parameters, inPixel, (float)z);
            point.setIncludingFrame(sensorFrame, outPoint[0], outPoint[1], outPoint[2]);
            point.changeFrame(ReferenceFrame.getWorldFrame());
            
            
            gp.setPosition(point);
            gp.update();
            gpIndex++;
         }
      }
   }
}
