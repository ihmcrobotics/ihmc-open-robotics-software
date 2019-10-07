package us.ihmc.atlas;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.stateEstimation.head.carnegie.multisense.MultisenseSLWithMicroStrainHeadPoseEstimator;
import us.ihmc.yoVariables.variable.YoFramePose3D;

import java.io.IOException;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class AtlasHeadPoseEstimator extends MultisenseSLWithMicroStrainHeadPoseEstimator
{
   private static final boolean DEBUG = false;
   private static final PriorityParameters imuListenerPriority = new PriorityParameters(20);

   private final String simpleName = getClass().getSimpleName();

   private final RigidBodyTransform headRigidBodyTransform = new RigidBodyTransform();
   private YoFramePose3D estimatedHeadPoseFramePoint;
   private YoGraphicCoordinateSystem estimatedHeadPoseViz;
   private YoGraphicCoordinateSystem imuFrame;

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber, boolean getRobotConfigurationDataFromNetwork)
         throws IOException
   {
      super(dt, MultisenseSLWithMicroStrainHeadPoseEstimator.DEFAULT_MULTISENSE_TO_IMU_TRANSFORM, imuListenerPriority, microStrainSerialNumber,
            getRobotConfigurationDataFromNetwork);

   }

   public AtlasHeadPoseEstimator(double dt, long microStrainSerialNumber)
         throws IOException
   {
      this(dt, microStrainSerialNumber, false);
   }

   @Override
   public void configureYoGraphics(YoGraphicsListRegistry parentYoGraphicListRegistry)
   {
      if (DEBUG)
      {
         YoGraphicsList graphicsList = new YoGraphicsList("AtlasHeadPoseEstimator");
         estimatedHeadPoseFramePoint = new YoFramePose3D("EstimatedHeadPoseFramePoint", ReferenceFrame.getWorldFrame(), getRegistry());

         estimatedHeadPoseViz = new YoGraphicCoordinateSystem("EstimatedHeadPoseVizualizer", estimatedHeadPoseFramePoint, 0.2, YoAppearance.DarkGray());

         imuFrame = new YoGraphicCoordinateSystem("HeadIMUFrame", new YoFramePose3D("HeadIMUPose", ReferenceFrame.getWorldFrame(), getRegistry()), 0.2);

         graphicsList.add(estimatedHeadPoseViz);
         graphicsList.add(imuFrame);

         parentYoGraphicListRegistry.registerYoGraphicsList(graphicsList);
      }
   }

   @Override
   public void compute()
   {
      super.compute();
      if (DEBUG)
      {
         getHeadTransform(headRigidBodyTransform);

         imuFrame.setToReferenceFrame(getImuFrame());

         if (estimatedHeadPoseViz != null)
         {
            estimatedHeadPoseFramePoint.set(headRigidBodyTransform);
         }
      }
   }
}
