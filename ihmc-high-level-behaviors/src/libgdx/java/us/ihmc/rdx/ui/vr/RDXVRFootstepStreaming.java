package us.ihmc.rdx.ui.vr;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.walkingController.SwingFootTracker;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;

public class RDXVRFootstepStreaming
{
   public static final boolean ENABLE_YO_VARIABLE_TOOLBOX_SERVER = false;

   private final RetargetingParameters retargetingParameters;
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames = new HashMap<>();

   private final RDXVRContext vrContext;
   private final RDXVRFootstepStreamingProcessor processor;
   private final ControllerStatusTracker controllerStatusTracker;
   private final SwingFootTracker swingFootTracker;
   private final SideDependentList<Float> gripButtonsValue = new SideDependentList<>();
   private boolean ankleTrackersPresent = false;

   public RDXVRFootstepStreaming(ROS2SyncedRobotModel syncedRobot,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 RDXVRContext vrContext,
                                 RetargetingParameters retargetingParameters,
                                 RDXVRFootstepPlacement footstepPlacer)
   {
      this.retargetingParameters = retargetingParameters;
      this.vrContext = vrContext;

      this.controllerStatusTracker = new ControllerStatusTracker(new LogToolsLogger(), ros2ControllerHelper.getROS2Node(), syncedRobot);
      this.swingFootTracker = new SwingFootTracker(syncedRobot, controllerStatusTracker);

      processor = new RDXVRFootstepStreamingProcessor(syncedRobot, ros2ControllerHelper, footstepPlacer, swingFootTracker, ENABLE_YO_VARIABLE_TOOLBOX_SERVER);
      RDXBaseUI.getInstance().getKeyBindings().register("Footstep Streaming: Control robot stepping (ankle trackers required)", "Hold both handle grippers");
   }

   public void processVRInput()
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller -> gripButtonsValue.put(side, controller.getGripActionData().x()));
      }
      Set<String> additionalTrackedSegments = vrContext.getAssignedTrackerRoles();
      boolean bothTrackersActive = true;
      for (VRTrackedSegmentType segmentType : FOOT_TYPES)
      {
         if (additionalTrackedSegments.contains(segmentType.getSegmentName()))
         {
            vrContext.getTracker(segmentType.getSegmentName()).runIfConnected(tracker ->
            {
               if (!trackerReferenceFrames.containsKey(segmentType.getSegmentName()))
               {
                  MutableReferenceFrame trackerDesiredControlFrame = new MutableReferenceFrame(tracker.getXForwardZUpTrackerFrame());
                  trackerDesiredControlFrame.getTransformToParent()
                                            .getRotation()
                                            .appendInvertOther(retargetingParameters.getControlFrameOrientationInBodyFrame(segmentType));
                  trackerDesiredControlFrame.getReferenceFrame().update();
                  trackerReferenceFrames.put(segmentType.getSegmentName(), trackerDesiredControlFrame);

                  processor.setTrackerReference(segmentType.getSegmentSide(), trackerDesiredControlFrame.getReferenceFrame());
               }

               processor.setTrackerVelocity(segmentType.getSegmentSide(),
                                            new SpatialVector(ReferenceFrame.getWorldFrame(),
                                                              tracker.getAngularVelocity(),
                                                              tracker.getLinearVelocity()));
               processor.setTrackerTimestamp(segmentType.getSegmentSide(), tracker.getLastPollTimeNanos());
            });
         }
         else
         {
            bothTrackersActive = false;
         }
      }

      if (bothTrackersActive)
      {
         boolean isStepping = gripButtonsValue.get(RobotSide.LEFT) > 0.8f && gripButtonsValue.get(RobotSide.RIGHT) > 0.8f;
         processor.processVRInput(isStepping);
      }
      ankleTrackersPresent = bothTrackersActive;
   }

   public void update()
   {
      if (ankleTrackersPresent)
      {
         swingFootTracker.update();
         processor.processToolboxOutput();
         // Stepping with ankle trackers pauses streaming until walking is done
         if (!controllerStatusTracker.isWalking())
         {
            if (processor.getReadyToStepNotification().poll())
            {
               processor.getReadyToStepNotification().clear();
               LogTools.warn("Stepping from VR");
               processor.step(false);
               controllerStatusTracker.getFinishedWalkingNotification().clear();
            }
         }
         else
         {
            if (processor.getReadyToStepNotification().poll())
            {
               LogTools.warn("Consecutive stepping from VR");
               processor.step(false);
               // This prevents wrong logic. The controller might think we're done walking even if we've just
               // sent a new footstep that needs to propagate to the controller
               controllerStatusTracker.getFinishedWalkingNotification().clear();
            }
         }
         }
   }

   public void reset()
   {
      trackerReferenceFrames.clear();
      processor.reset();
   }

   public void destroy()
   {
      if (processor != null)
      {
         processor.destroy();
      }
   }
}