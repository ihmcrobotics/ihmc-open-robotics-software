package us.ihmc.rdx.ui.vr;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;
import static us.ihmc.rdx.ui.vr.RDXVRKinematicsStreamingMode.FRAME_AXIS_GRAPHICS_LENGTH;

/**
 * Class responsible for motion retargeting from VR tracked segments to a robot model.
 * It adjusts the robot's pelvis and center of mass based on the tracked segments.
 */
public class RDXVRMotionRetargeting
{
   // Set of trackers whose references are unchanged during retargeting
   private final Set<VRTrackedSegmentType> UNCHANGED_TRACKER_REFERENCES = new HashSet<>()
   {
      {
         add(CHEST);
         add(LEFT_WRIST);
         add(RIGHT_WRIST);
      }
   };
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<MutableReferenceFrame> controllerReferenceFrames;
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames;
   private final MutableReferenceFrame headsetReferenceFrame;
   private final RetargetingParameters retargetingParameters;

   private final RigidBodyTransform initialWaistTrackerTransformToWorld = new RigidBodyTransform();
   private final FramePose3D newPelvisFramePose = new FramePose3D();
   private final Map<VRTrackedSegmentType, ReferenceFrame> retargetedFrames = new HashMap<>();
   private final RigidBodyTransform initialPelvisTransformToWorld = new RigidBodyTransform();
   private Point3D centerOfMassDesiredXYInWorld;
   private ReferenceFrame initialPelvisFrame;
   private ReferenceFrame scaledPelvisFrame;
   private final SideDependentList<Point3D> initialHandPositionsInWorld = new SideDependentList<>(null, null);
   private boolean controlArmsOnly = false;
   private boolean armScaling = false;
   private boolean comTracking = false;
   private double armLengthScaleFactor = 1.0;
   private final SideDependentList<RDXReferenceFrameGraphic> shoulderFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> scaledHandsFrameGraphics = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> shoulderFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> scaledHandFrames = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> shoulderToScaledHandTransforms = new SideDependentList<>();

   /**
    * Constructor for the motion retargeting class.
    *
    * @param syncedRobot the synchronized robot model
    * @param controllerReferenceFrames the reference frames of the controllers
    * @param trackerReferenceFrames the reference frames of the trackers
    * @param retargetingParameters the retargeting parameters
    */
   public RDXVRMotionRetargeting(ROS2SyncedRobotModel syncedRobot,
                                 SideDependentList<MutableReferenceFrame> controllerReferenceFrames,
                                 Map<String, MutableReferenceFrame> trackerReferenceFrames,
                                 MutableReferenceFrame headsetReferenceFrame,
                                 RetargetingParameters retargetingParameters)
   {
      this.syncedRobot = syncedRobot;
      this.retargetingParameters = retargetingParameters;
      this.controllerReferenceFrames = controllerReferenceFrames;
      this.trackerReferenceFrames = trackerReferenceFrames;
      this.headsetReferenceFrame = headsetReferenceFrame;

      for (RobotSide side : RobotSide.values)
      {
         shoulderFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         scaledHandsFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         shoulderToScaledHandTransforms.put(side, new RigidBodyTransform());
      }
   }

   /**
    * Computes the desired values for retargeting.
    * Updates the pelvis and center of mass.
    */
   public void computeDesiredValues()
   {
      retargetedFrames.clear();
      retargetPelvis();
      retargetCoM();
      retargetHands();
   }

   /**
    * Compute the desired pose of pelvis.
    * Scale the pelvis height based on ratio user/robot height and use incremental retargeting.
    */
   private void retargetPelvis()
   {
      if (trackerReferenceFrames.containsKey(WAIST.getSegmentName()) && !controlArmsOnly)
      {
         if (initialPelvisFrame == null)
         {
            initialPelvisTransformToWorld.set(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
            initialPelvisFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                          initialPelvisTransformToWorld);
            initialWaistTrackerTransformToWorld.set(trackerReferenceFrames.get(WAIST.getSegmentName()).getReferenceFrame().getTransformToWorldFrame());

            scaledPelvisFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), newPelvisFramePose);
         }
         // Calculate the variation of the tracker's frame from its initial value
         RigidBodyTransform waistTrackerVariationFromInitialValue = new RigidBodyTransform(trackerReferenceFrames.get(WAIST.getSegmentName())
                                                                                                                 .getReferenceFrame()
                                                                                                                 .getTransformToWorldFrame());
         // Get variation from initial value
         initialWaistTrackerTransformToWorld.inverseTransform(waistTrackerVariationFromInitialValue);
         // Scale the z waist variation
         double scalingRobotHumanWaistHeight = retargetingParameters.getPelvisHeightExtendedLegs() / initialWaistTrackerTransformToWorld.getTranslationZ();
         waistTrackerVariationFromInitialValue.getTranslation()
                                              .setZ(scalingRobotHumanWaistHeight * waistTrackerVariationFromInitialValue.getTranslationZ());

         // Concatenate the initial pelvis transform with the variation
         RigidBodyTransform combinedTransformToWorld = new RigidBodyTransform(initialPelvisTransformToWorld);
         combinedTransformToWorld.multiply(waistTrackerVariationFromInitialValue);

         newPelvisFramePose.set(combinedTransformToWorld);
         // Zero roll orientation variation as it can lead to very unnatural motions (at least when in double support)
         newPelvisFramePose.changeFrame(initialPelvisFrame);
         newPelvisFramePose.getRotation().setYawPitchRoll(newPelvisFramePose.getRotation().getYaw(), newPelvisFramePose.getRotation().getPitch(),0.0);
         newPelvisFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         scaledPelvisFrame.update();

         retargetedFrames.put(WAIST, scaledPelvisFrame);
      }
   }

   /**
    * Compute desired CoM for the robot based on ankles and waist tracker on user.
    * Use center of mass normalized offset computation.
    */
   private void retargetCoM()
   {
      if (comTracking)
      {
         if (trackerReferenceFrames.containsKey(WAIST.getSegmentName()) && trackerReferenceFrames.containsKey(LEFT_ANKLE.getSegmentName())
             && trackerReferenceFrames.containsKey(RIGHT_ANKLE.getSegmentName()) && !controlArmsOnly)
         {
            if (centerOfMassDesiredXYInWorld == null)
            {
               centerOfMassDesiredXYInWorld = new Point3D();
               centerOfMassDesiredXYInWorld.set(syncedRobot.getReferenceFrames().getMidFeetZUpFrame().getTransformToWorldFrame().getTranslation());
            }
            // Fetch the current frames for left and right ankle
            ReferenceFrame leftAnkleFrame = trackerReferenceFrames.get(LEFT_ANKLE.getSegmentName()).getReferenceFrame();
            ReferenceFrame rightAnkleFrame = trackerReferenceFrames.get(RIGHT_ANKLE.getSegmentName()).getReferenceFrame();

            // Get the waist tracker ground projection of the user (assuming this can approximate the ground projection of the center of mass)
            Point3D waistTrackerXYInWorld = new Point3D(trackerReferenceFrames.get(WAIST.getSegmentName()).getReferenceFrame().getTransformToWorldFrame().getTranslation());
            waistTrackerXYInWorld.setZ(0.0);

            // Get the positions of the left and right foot on the ground
            Point3D leftAnkleTrackerXYInWorld = new Point3D(leftAnkleFrame.getTransformToWorldFrame().getTranslation());
            leftAnkleTrackerXYInWorld.setZ(0.0);
            Point3D rightAnkleTrackerXYInWorld = new Point3D(rightAnkleFrame.getTransformToWorldFrame().getTranslation());
            rightAnkleTrackerXYInWorld.setZ(0.0);

            // Calculate the vector between the ankles (feet)
            Vector3D virtualFeetVector = new Vector3D();
            virtualFeetVector.sub(rightAnkleTrackerXYInWorld, leftAnkleTrackerXYInWorld);

            // Project the waist tracker ground projection onto the line between the feet
            Vector3D projectionVector = new Vector3D(waistTrackerXYInWorld);
            projectionVector.sub(leftAnkleTrackerXYInWorld);
            double dotProduct = projectionVector.dot(virtualFeetVector);
            double normSqFeetVector = virtualFeetVector.lengthSquared();

            // Calculate normalized offset along the line connecting the feet
            double normalizedOffset = dotProduct / normSqFeetVector;
            // Filter value
            double filteredNormalizedOffset = normalizedOffset;
            if (filteredNormalizedOffset >= 1.0)
               filteredNormalizedOffset = 1.0;
            else if (filteredNormalizedOffset <= 0.0)
            {
               filteredNormalizedOffset = 0.0;
            }
            //         else
            //         { //logit function
            //            filteredNormalizedOffset = 0.5- 0.1 * (Math.log10((1 - normalizedOffset) / normalizedOffset));
            //         }

            Point3D leftFootXYInWorld = new Point3D(syncedRobot.getFullRobotModel().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getTranslation());
            Point3D rightFootXYInWorld = new Point3D(syncedRobot.getFullRobotModel().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame().getTranslation());

            // Reconstruct robot CoM based on the normalized offset and its feet position
            Vector3D feetVector = new Vector3D();
            feetVector.sub(rightFootXYInWorld, leftFootXYInWorld);

            centerOfMassDesiredXYInWorld.set(feetVector);
            centerOfMassDesiredXYInWorld.scale(filteredNormalizedOffset);
            centerOfMassDesiredXYInWorld.add(leftFootXYInWorld);
         }
      }
   }

   /**
    * Compute desired pose of hands.
    * Use the pose of the chest tracker (on the stern of the user) and the headset location to estimate the position of the shoulders.
    * Then scale the relative position of the hand wrt to the shoulder based on the ratio of robot arm length / initial controller frame.
    */
   private void retargetHands()
   {
      if (armScaling)
      {
         if (trackerReferenceFrames.containsKey(CHEST.getSegmentName()) && headsetReferenceFrame != null)
         {
            ReferenceFrame chestTrackerFrame = trackerReferenceFrames.get(CHEST.getSegmentName()).getReferenceFrame();
            ReferenceFrame headsetFrame = headsetReferenceFrame.getReferenceFrame();

            // Estimate shoulder positions based on chest and headset positions
            Point3D chestTrackerPosition = new Point3D(chestTrackerFrame.getTransformToWorldFrame().getTranslation());
            Point3D headsetPosition = new Point3D(headsetFrame.getTransformToWorldFrame().getTranslation());

            // Scale the controller reference frames
            for (RobotSide side : RobotSide.values())
            {
               ReferenceFrame handFrame = controllerReferenceFrames.get(side).getReferenceFrame();
               if (initialHandPositionsInWorld.get(side) == null)
               {
                  // Calculate Z offset as the midpoint between the chest and headset positions
                  double shoulderOffsetZ = (headsetPosition.getZ() - chestTrackerPosition.getZ()) / 2.0;
                  // Calculate head length based on the sternum to headset distance
                  double distanceChestToHeadset = chestTrackerPosition.distance(headsetPosition);
                  double headLength = distanceChestToHeadset / 1.5;
                  // The Y offset for the shoulders is equal to the head length
                  double shoulderOffsetY = headLength;

                  // Create the shoulder frame
                  RigidBodyTransform shoulderToChestTransform = new RigidBodyTransform(new YawPitchRoll(), new Point3D(0.0, side.negateIfRightSide(shoulderOffsetY), shoulderOffsetZ));
                  shoulderFrames.put(side, ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(chestTrackerFrame, shoulderToChestTransform));
                  shoulderFrameGraphics.get(side).setToReferenceFrame(shoulderFrames.get(side));
                  // In theory, we should use robot shoulder frame as parent frame, but that creates a feedback loop that causes instability
                  // Using the reconstructed user shoulder frame is close enough
                  scaledHandFrames.put(side, ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(shoulderFrames.get(side),
                                                                                                                    shoulderToScaledHandTransforms.get(side)));

                  initialHandPositionsInWorld.put(side, new Point3D(handFrame.getTransformToWorldFrame().getTranslation()));
                  // Calculate initial user arm length
                  double userArmLength = shoulderFrames.get(side).getTransformToWorldFrame().getTranslationZ() - initialHandPositionsInWorld.get(side).getZ();
                  // Scale factor for arm length
                  double robotArmLength = retargetingParameters.getArmLength();
                  armLengthScaleFactor = robotArmLength / userArmLength;
               }

               Vector3DReadOnly shoulderPosition = shoulderFrames.get(side).getTransformToWorldFrame().getTranslation();
               // Compute scaled hand position
               Vector3D scaledArmVector = new Vector3D();
               scaledArmVector.sub(handFrame.getTransformToWorldFrame().getTranslation(), shoulderPosition);
               scaledArmVector.scale(armLengthScaleFactor);

               shoulderToScaledHandTransforms.get(side).getTranslation().set(scaledArmVector);
               RotationMatrixReadOnly controllerOrientation = controllerReferenceFrames.get(side)
                                                                                       .getReferenceFrame()
                                                                                       .getTransformToDesiredFrame(scaledHandFrames.get(side).getParent())
                                                                                       .getRotation();
               shoulderToScaledHandTransforms.get(side).getRotation().set(controllerOrientation);
               scaledHandFrames.get(side).update();

               // Update graphics
               shoulderFrameGraphics.get(side).setToReferenceFrame(shoulderFrames.get(side));
               scaledHandsFrameGraphics.get(side).setToReferenceFrame(scaledHandFrames.get(side));

               // Set desired frame for hand
               retargetedFrames.put(side == RobotSide.LEFT ? LEFT_HAND : RIGHT_HAND, scaledHandFrames.get(side));
            }
         }
      }
   }

   public void setControlArmsOnly(boolean controlArmsOnly)
   {
      this.controlArmsOnly = controlArmsOnly;
   }

   public void setArmScaling(boolean armScaling)
   {
      this.armScaling = armScaling;
   }

   public void setCoMTracking(boolean comTracking)
   {
      this.comTracking = comTracking;
   }

   public void reset()
   {
      initialPelvisFrame = null;
      for (RobotSide side : RobotSide.values)
         initialHandPositionsInWorld.put(side, null);
      retargetedFrames.clear();
      centerOfMassDesiredXYInWorld = null;
   }

   public Set<VRTrackedSegmentType> getRetargetedSegments()
   {
      return retargetedFrames.keySet();
   }

   public ReferenceFrame getDesiredFrame(VRTrackedSegmentType segment)
   {
      return retargetedFrames.get(segment);
   }

   public Point3D getDesiredCenterOfMassXYInWorld()
   {
      return centerOfMassDesiredXYInWorld;
   }

   public boolean isCenterOfMassAvailable()
   {
      return centerOfMassDesiredXYInWorld != null;
   }

   public boolean isRetargetingNotNeeded(VRTrackedSegmentType segment)
   {
      return UNCHANGED_TRACKER_REFERENCES.contains(segment);
   }

   public RDXReferenceFrameGraphic getShoulderGraphic(RobotSide side)
   {
      return shoulderFrameGraphics.get(side);
   }

   public RDXReferenceFrameGraphic getScaledHandGraphic(RobotSide side)
   {
      return scaledHandsFrameGraphics.get(side);
   }
}
