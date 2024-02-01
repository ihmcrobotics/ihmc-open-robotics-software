package us.ihmc.perception.realsense.example;

import java.nio.ShortBuffer;
import java.util.ArrayList;

import org.bytedeco.librealsense2.rs2_intrinsics;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

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
   
   private final YoBoolean enableUserSensorFrame;
   private final PoseReferenceFrame userSensorFrame;
   private final YoFramePoseUsingYawPitchRoll userSensorPose;
   
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
            YoGraphicPosition gp = new YoGraphicPosition(prefix + x + "x" + y + "y", "", registry, 0.005, YoAppearance.Red());
            graphicsListRegistry.registerYoGraphic("l515DepthPoints", gp);
            positions.add(gp);
         }
      }
      
      enableUserSensorFrame = new YoBoolean("enableUserSensorFrame", registry);
      userSensorPose = new YoFramePoseUsingYawPitchRoll("userSensorPose", sensorFrame.getParent(), registry);
      userSensorPose.setFromReferenceFrame(sensorFrame);
      
      userSensorPose.setX(0.160);
      userSensorPose.setY(-0.085);
      userSensorPose.setZ(-0.052);
      userSensorPose.setYaw(-2.61799);
      userSensorPose.setPitch(0.261799);
      userSensorPose.setRoll(3.08923);
      
      userSensorFrame = new PoseReferenceFrame("userSensorFrame", userSensorPose);
   }
   
   float[] outPoint = new float[3];
   float[] inPixel = new float[2];
   FramePoint3D point = new FramePoint3D();
   
   public void update(ShortBuffer depthData, double depthToMeterConversion, rs2_intrinsics intrinsic_parameters)
   {
      userSensorFrame.setPoseAndUpdate(userSensorPose);
      
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
            
            if(enableUserSensorFrame.getBooleanValue())
            {
               point.setIncludingFrame(userSensorFrame, outPoint[0], outPoint[1], outPoint[2]);
            }
            else
            {
               point.setIncludingFrame(sensorFrame, outPoint[0], outPoint[1], outPoint[2]);
            }
            
            point.changeFrame(ReferenceFrame.getWorldFrame());
            
            
            gp.setPosition(point);
            gp.update();
            gpIndex++;
         }
      }
   }
}
