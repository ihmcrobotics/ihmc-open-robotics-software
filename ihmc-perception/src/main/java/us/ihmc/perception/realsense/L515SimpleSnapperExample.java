package us.ihmc.perception.realsense;

import static org.bytedeco.librealsense2.global.realsense2.RS2_EXTENSION_DEPTH_FRAME;
import static org.bytedeco.librealsense2.global.realsense2.rs2_is_frame_extendable_to;

import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.Random;

import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class L515SimpleSnapperExample
{
   private final boolean VISUALIZE_POINT_CLOUD = false;

   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;

   private RealtimeL515 l515;
   private final rs2_error e = new rs2_error();

   //Temp Pointers
   private final FramePoint3D workingPoint = new FramePoint3D();
   private final float[] pointInCameraSpace = new float[3];
   private final float[] pointInPixelSpace = new float[2];

   private final PoseReferenceFrame sensorFrame;
   private final YoFramePoseUsingYawPitchRoll sensorPose;

   private final PoseReferenceFrame desiredFootstepFrame;
   private final YoFramePoseUsingYawPitchRoll desiredFootstepPose;

   private final ArrayList<YoFramePoint3D> sampleLocations = new ArrayList<YoFramePoint3D>();
   private final ArrayList<YoFramePoint3D> updatedLocations = new ArrayList<YoFramePoint3D>();

   //YoGraphics
   private final YoGraphicReferenceFrame desiredFootstepFrameGraphic;
   private final YoGraphicReferenceFrame sensorFrameGraphic;
   private L515Visualizer viz;

   public L515SimpleSnapperExample(String prefix, RealtimeL515 l515, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.l515 = l515;

      registry = new YoRegistry(prefix + name);

      sensorFrame = new PoseReferenceFrame(prefix + "sensorFrame", ReferenceFrame.getWorldFrame());
      sensorPose = new YoFramePoseUsingYawPitchRoll("sensorPose", ReferenceFrame.getWorldFrame(), registry);
      sensorFrameGraphic = new YoGraphicReferenceFrame(sensorFrame, registry, true, 0.2);

      desiredFootstepFrame = new PoseReferenceFrame("desiredFootstepFrame", ReferenceFrame.getWorldFrame());
      desiredFootstepPose = new YoFramePoseUsingYawPitchRoll("desiredFootstepPose", ReferenceFrame.getWorldFrame(), registry);
      desiredFootstepFrameGraphic = new YoGraphicReferenceFrame(desiredFootstepFrame, registry, true, 0.2);

      graphicsListRegistry.registerYoGraphic("l515Points", sensorFrameGraphic);
      graphicsListRegistry.registerYoGraphic("l515Points", desiredFootstepFrameGraphic);

      Random random = new Random();
      YoFramePoint3D updatedCenterPoint = new YoFramePoint3D(prefix + "_centerUpdatedPoint", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D sampleCenterPoint = new YoFramePoint3D(prefix + "_centerSamplePoint", desiredFootstepFrame, registry);

      updatedLocations.add(updatedCenterPoint);
      sampleLocations.add(sampleCenterPoint);

      YoGraphicPosition updatedPositionGraphic = new YoGraphicPosition(prefix + "centerUpdatedPoint",
                                                                       updatedCenterPoint,
                                                                       0.01,
                                                                       YoAppearance.randomColor(random));

      graphicsListRegistry.registerYoGraphic("l515Points", updatedPositionGraphic);

      for (RobotQuadrant footQuadrant : RobotQuadrant.values)
      {
         YoFramePoint3D samplePoint = new YoFramePoint3D(prefix + footQuadrant.getUnderBarName() + "samplePoint", desiredFootstepFrame, registry);
         samplePoint.setX(footQuadrant.getSide().negateIfRightSide(0.06));
         samplePoint.setY(footQuadrant.getEnd().negateIfHindEnd(0.06));
         sampleLocations.add(samplePoint);

         YoFramePoint3D updatedPoint = new YoFramePoint3D(prefix + footQuadrant.getUnderBarName() + "updatedPoint", ReferenceFrame.getWorldFrame(), registry);
         updatedLocations.add(updatedPoint);

         YoGraphicPosition updatedQuadPositionGraphic = new YoGraphicPosition(prefix + footQuadrant.getUnderBarName() + "updatedPoint",
                                                                              updatedPoint,
                                                                              0.01,
                                                                              YoAppearance.randomColor(random));

         graphicsListRegistry.registerYoGraphic("l515Points", updatedQuadPositionGraphic);
      }

      if (VISUALIZE_POINT_CLOUD)
      {
         viz = new L515Visualizer(prefix, l515.getDepthWidth(), l515.getDepthHeight(), 48, 40, sensorFrame, registry, graphicsListRegistry);
      }

      //Manually setting the sensor pose for debugging
      //The camera was mounted to a table, pointing to the ground
      sensorPose.setZ(1.0922);
      sensorPose.appendRollRotation(3.5);
   }

   public void receivedDepthData(rs2_frame depthFrame, ShortBuffer depthData)
   {
      sensorFrame.setPoseAndUpdate(sensorPose);
      sensorFrameGraphic.update();
      desiredFootstepFrameGraphic.update();

      getHeightAtStepFromDepthData(depthData);

      if (VISUALIZE_POINT_CLOUD)
      {
         viz.update(depthData, l515.getDepthToMeterConversion(), l515.getIntrinsicParameters());
      }
   }

   /**
    * This was an attempt to get the distance of a particular pixel without getting the depth data. I
    * could not find a way of doing this in a garbage free way, but I suspect it's faster
    */
   public void getHeightAtStepFromFrame(rs2_frame depthFrame)
   {
      // Check if the given frame can be extended to depth frame interface
      if (0 == rs2_is_frame_extendable_to(depthFrame, RS2_EXTENSION_DEPTH_FRAME, e))
         return;

      desiredFootstepFrame.setPoseAndUpdate(desiredFootstepPose);

      for (int i = 0; i < sampleLocations.size(); i++)
      {
         YoFramePoint3D samplePoint = sampleLocations.get(i);
         workingPoint.setIncludingFrame(samplePoint);
         workingPoint.changeFrame(sensorFrame);

         workingPoint.get(pointInCameraSpace);
         realsense2.rs2_project_point_to_pixel(pointInPixelSpace, l515.getIntrinsicParameters(), pointInCameraSpace);
         float distance = realsense2.rs2_depth_frame_get_distance(depthFrame, (int) pointInPixelSpace[0], (int) pointInPixelSpace[1], e);
      }
   }

   /**
    * Given the depth data from the latest depth frame and a desired footstep location get the
    * different Z heights at the specified sample locations TODO: add parameter to take in footsteps to
    * snap
    */
   public void getHeightAtStepFromDepthData(ShortBuffer depthData)
   {
      //The desiredFootstepPose is a yovariable pose and is currently being set from SCS for testing
      desiredFootstepFrame.setPoseAndUpdate(desiredFootstepPose);

      //iterate through the different sample locations around the desired footstep
      for (int i = 0; i < sampleLocations.size(); i++)
      {
         YoFramePoint3D samplePoint = sampleLocations.get(i);
         YoFramePoint3D updatedPoint = updatedLocations.get(i);

         workingPoint.setIncludingFrame(samplePoint);
         workingPoint.changeFrame(sensorFrame);

         workingPoint.get(pointInCameraSpace);
         //project the sample position into camera space and get the associated pixels at those locations
         //SEE https://dev.intelrealsense.com/docs/projection-in-intel-realsense-sdk-20 for more information
         realsense2.rs2_project_point_to_pixel(pointInPixelSpace, l515.getIntrinsicParameters(), pointInCameraSpace);

         int x = (int) pointInPixelSpace[0];
         int y = (int) pointInPixelSpace[1];

         //only try to adjust stuff within our FoV
         if (x > 0 && x < l515.getDepthWidth() && y > 0 && y < l515.getDepthHeight())
         {
            int index = y * l515.getDepthWidth() + x;
            //Explicit call to deal with unsigned numbers. Java doesn't like them
            int val = Short.toUnsignedInt(depthData.get(index));

            double distance = val * l515.getDepthToMeterConversion();

            pointInPixelSpace[0] = x;
            pointInPixelSpace[1] = y;

            //Just in case, reproject the point back to 3D space. Not sure if this hurts or helps
            realsense2.rs2_deproject_pixel_to_point(pointInCameraSpace, l515.getIntrinsicParameters(), pointInPixelSpace, (float) distance);

            //Convert the point back into world, though it seems safer to go back to pelvis...
            workingPoint.setIncludingFrame(sensorFrame, pointInCameraSpace[0], pointInCameraSpace[1], pointInCameraSpace[2]);
            workingPoint.changeFrame(ReferenceFrame.getWorldFrame());

            //result goes in a yovariable for testing
            updatedPoint.set(workingPoint);
         }
      }

      //TODO: decide whether to average or take highest points and throw out invalid data
   }
}
