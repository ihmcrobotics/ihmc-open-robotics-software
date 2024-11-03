package us.ihmc.perception.realsense.example;

import static org.bytedeco.librealsense2.global.realsense2.RS2_EXTENSION_DEPTH_FRAME;
import static org.bytedeco.librealsense2.global.realsense2.rs2_is_frame_extendable_to;

import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.Random;

import controller_msgs.msg.dds.FootstepDataMessage;
import org.bytedeco.librealsense2.rs2_error;
import org.bytedeco.librealsense2.rs2_frame;
import org.bytedeco.librealsense2.global.realsense2;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class L515SimpleSnapperExample implements FootstepAdjustment, L515DepthImageReceiver
{
   private final boolean VISUALIZE_POINT_CLOUD = true;

   protected final String name = getClass().getSimpleName();
   protected final YoRegistry registry;

   private RealtimeL515 l515;
   private final rs2_error e = new rs2_error();

   //Temp Pointers
   private final FramePoint3D workingPoint = new FramePoint3D();
   private final float[] pointInCameraSpace = new float[3];
   private final float[] pointInPixelSpace = new float[2];
   
   private ConcurrentCopier<short[]> concurrentCopier;
   private final int depthSize;

   private final ReferenceFrame sensorFrame;

   private final PoseReferenceFrame desiredFootstepFrame;
   private final YoFramePoseUsingYawPitchRoll desiredFootstepPose;

   private final ArrayList<YoFramePoint3D> sampleLocations = new ArrayList<YoFramePoint3D>();
   private final ArrayList<YoFramePoint3D> updatedLocations = new ArrayList<YoFramePoint3D>();
   
   private final FramePose3D updatedFootStepPose = new FramePose3D();

   //YoGraphics
   private final YoGraphicReferenceFrame desiredFootstepFrameGraphic;
   private final YoGraphicReferenceFrame sensorFrameGraphic;
   private L515Visualizer viz;

   private SideDependentList<MovingReferenceFrame> soleFrames;

   public L515SimpleSnapperExample(String prefix, FullHumanoidRobotModel fullHumanoidRobotModel, RealtimeL515 l515, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.l515 = l515;
      soleFrames = fullHumanoidRobotModel.getSoleFrames();
      registry = new YoRegistry(prefix + name);
      depthSize = l515.getDepthWidth() * l515.getDepthHeight();
      
      concurrentCopier = new ConcurrentCopier<short[]>(new Builder<short[]>()
      {
         @Override
         public short[] newInstance()
         {
            return new short[depthSize];
         }
      });
      
      this.sensorFrame = l515.getSensorFrame();
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
      
      parentRegistry.addChild(registry);
   }

   public void receivedDepthData(rs2_frame depthFrame, ShortBuffer depthDataBuffer)
   {
      depthDataBuffer.position(0);
      depthDataBuffer.get(concurrentCopier.getCopyForWriting(), 0, depthSize);
      concurrentCopier.commit();

      if (VISUALIZE_POINT_CLOUD)
      {
         viz.update(depthDataBuffer, l515.getDepthToMeterConversion(), l515.getIntrinsicParameters());
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
    * different Z heights at the specified sample locations 
    */
   @Override
   public boolean adjustFootstep(FramePose3DReadOnly stancePose, FramePose2DReadOnly footstepPose, RobotSide footSide, FootstepDataMessage adjustedFootstep)
   {
      desiredFootstepPose.set(footstepPose);
      desiredFootstepFrame.setPoseAndUpdate(desiredFootstepPose);

      sensorFrameGraphic.update();
      desiredFootstepFrameGraphic.update();
      
      YoFramePoint3D highestPoint = null;
      
      //iterate through the different sample locations around the desired footstep and get the max z height
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

         short[] depthData = concurrentCopier.getCopyForReading();
         
         //only try to adjust stuff within our FoV
         if (x > 0 && x < l515.getDepthWidth() && y > 0 && y < l515.getDepthHeight())
         {
            int index = y * l515.getDepthWidth() + x;
            
            //Explicit call to deal with unsigned numbers. Java doesn't like them
            int val = Short.toUnsignedInt(depthData[index]);

            double distance = val * l515.getDepthToMeterConversion();

            pointInPixelSpace[0] = x;
            pointInPixelSpace[1] = y;

            //Just in case, reproject the point back to 3D space. Not sure if this hurts or helps
            realsense2.rs2_deproject_pixel_to_point(pointInCameraSpace, l515.getIntrinsicParameters(), pointInPixelSpace, (float) distance);

            //Convert the point back into world
            workingPoint.setIncludingFrame(sensorFrame, pointInCameraSpace[0], pointInCameraSpace[1], pointInCameraSpace[2]);
            workingPoint.changeFrame(ReferenceFrame.getWorldFrame());

            //result goes in a yovariable for debugging
            updatedPoint.set(workingPoint);
            
            if(highestPoint == null)
            {
               highestPoint = updatedPoint;
            }
            
            if(updatedPoint.getZ() > highestPoint.getZ())
            {
               highestPoint = updatedPoint;
            }
         }
      }
      
      if (highestPoint == null)
      {
         return false;
      }
        
      if(highestPoint.getZ() < adjustedFootstep.getLocation().getZ() + 0.3)
      {
         //just update the Z height
         updatedFootStepPose.setIncludingFrame(footstepPose);
         updatedFootStepPose.getPosition().setZ(highestPoint.getZ());
         
         adjustedFootstep.getLocation().set(updatedFootStepPose.getPosition());
         adjustedFootstep.getOrientation().set(updatedFootStepPose.getOrientation());
         return true;
      }
      return false;

      //TODO: Throw out invalid data, update the foot orientation, and do a simple foot wiggle
   }
   
   /* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
   private void rs2_deproject_pixel_to_point(float[] point, Intrinsics intrin, float[] pixel, float depth)
   {
//       assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
//       assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
       //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

       float x = (pixel[0] - intrin.getPpx()) / intrin.getFx();
       float y = (pixel[1] - intrin.getPpy()) / intrin.getFy();

       if (intrin.getModel() == realsense2.RS2_DISTORTION_INVERSE_BROWN_CONRADY)
       {
          float[] coeffs = intrin.getCoeffs();
          
           float r2 = x * x + y * y;
           float f = 1 + coeffs[0] * r2 + coeffs[1] * r2*r2 + coeffs[4] * r2*r2*r2;
           float ux = x * f + 2 * coeffs[2] * x*y + coeffs[3] * (r2 + 2 * x*x);
           float uy = y * f + 2 * coeffs[3] * x*y + coeffs[2] * (r2 + 2 * y*y);
           x = ux;
           y = uy;
       }
       point[0] = depth * x;
       point[1] = depth * y;
       point[2] = depth;
   }
   
   /* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera */
   private void rs2_project_point_to_pixel(float[] pixel,  Intrinsics intrin,  float[] point)
   {
       //assert(intrin->model != RS2_DISTORTION_INVERSE_BROWN_CONRADY); // Cannot project to an inverse-distorted image

       float x = point[0] / point[2], y = point[1] / point[2];

       if (intrin.getModel() == realsense2.RS2_DISTORTION_INVERSE_BROWN_CONRADY)
       {
           float[] coeffs = intrin.getCoeffs();
           float r2 = x * x + y * y;
           float f = 1 + coeffs[0] * r2 + coeffs[1] * r2*r2 + coeffs[4] * r2*r2*r2;
           x *= f;
           y *= f;
           float dx = x + 2 * coeffs[2] * x*y + coeffs[3] * (r2 + 2 * x*x);
           float dy = y + 2 * coeffs[3] * x*y + coeffs[2] * (r2 + 2 * y*y);
           x = dx;
           y = dy;
       }

       if (intrin.getModel() == realsense2.RS2_DISTORTION_FTHETA)
       {
           float[] coeffs = intrin.getCoeffs();
           double r = Math.sqrt(x*x + y * y);
           double rd = (float)(1.0 / coeffs[0] * Math.atan(2 * r* Math.tan(coeffs[0] / 2.0)));
           x *= rd / r;
           y *= rd / r;
       }

       pixel[0] = x * intrin.getFx() + intrin.getPpx();
       pixel[1] = y * intrin.getFy() + intrin.getPpy();
   }
   
   private class Intrinsics
   {
      private float width, height;
      private float ppx, ppy, fx, fy;
      private float[] coeffs = new float[5];
      private int model;
      
      /** Width of the image in pixels */
      public float getWidth()
      {
         return width;
      }
      public void setWidth(float width)
      {
         this.width = width;
      }
      
      /** Height of the image in pixels */
      public float getHeight()
      {
         return height;
      }
      public void setHeight(float height)
      {
         this.height = height;
      }

      /** Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
      public float getPpx()
      {
         return ppx;
      }
      public void setPpx(float ppx)
      {
         this.ppx = ppx;
      }

      /** Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
      public float getPpy()
      {
         return ppy;
      }
      public void setPpy(float ppy)
      {
         this.ppy = ppy;
      }

      /** Focal length of the image plane, as a multiple of pixel width */
      public float getFx()
      {
         return fx;
      }
      public void setFx(float fx)
      {
         this.fx = fx;
      }

      /** Focal length of the image plane, as a multiple of pixel height */
      public float getFy()
      {
         return fy;
      }
      public void setFy(float fy)
      {
         this.fy = fy;
      }

      /** Distortion coefficients. Order for Brown-Conrady: [k1, k2, p1, p2, k3]. Order for F-Theta Fish-eye: [k1, k2, k3, k4, 0]. Other models are subject to their own interpretations */
      public float[] getCoeffs()
      {
         return coeffs;
      }
      public void setCoeffs(float[] coeffs)
      {
         for(int i = 0; i < coeffs.length; i++)
         {
            this.coeffs[i] = coeffs[i];
         }
      }
      public int getModel()
      {
         return model;
      }
      public void setModel(int model)
      {
         this.model = model;
      }
  }
}
