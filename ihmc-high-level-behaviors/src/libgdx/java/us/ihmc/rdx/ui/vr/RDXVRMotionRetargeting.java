package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import static us.ihmc.motionRetargeting.VRTrackedSegmentType.*;
import static us.ihmc.rdx.ui.vr.RDXVRWholeBodyKinematicStreaming.FRAME_AXIS_GRAPHICS_LENGTH;

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
   private final FullHumanoidRobotModel realRobotModel;
   private final FullHumanoidRobotModel ghostRobotModel;
   private final SideDependentList<MutableReferenceFrame> controllerReferenceFrames;
   private final Map<String, MutableReferenceFrame> trackerReferenceFrames;
   private final MutableReferenceFrame headsetReferenceFrame;
   private final RetargetingParameters retargetingParameters;
   private final Map<VRTrackedSegmentType, ReferenceFrame> retargetedFrames = new HashMap<>();
   private final Map<VRTrackedSegmentType, Point3D> ikControlFrameOffsetPosition = new HashMap<>();
   private final Map<VRTrackedSegmentType, FrameVector3D> desiredLinearVelocities = new HashMap<>();
   private final Map<VRTrackedSegmentType, FrameVector3D> desiredAngularVelocities = new HashMap<>();

   private final RigidBodyTransform initialPelvisTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame initialPelvisFrame;
   private ReferenceFrame constrainedPelvisFrame;
   private final FramePose3D newPelvisFramePose = new FramePose3D();

   private boolean comTracking = true;
   private Point3D centerOfMassDesiredXYInWorld;
   private double previousOffsetValue = 0.5;
   private final SideDependentList<Point3D> initialHandPositionsInWorld = new SideDependentList<>();
   private boolean controlArmsOnly = false;
   private boolean armScaling = false;
   private double armLengthScaleFactor = 1.0;
   private final SideDependentList<RDXReferenceFrameGraphic> shoulderFrameGraphics = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> scaledHandsFrameGraphics = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> shoulderFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> scaledHandFrames = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> shoulderToScaledHandTransforms = new SideDependentList<>();

   private final SideDependentList<RigidBodyTransform> initialFootTrackerTransformsToWorld = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> initialFootTransformsToWorld = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> initialFootFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> constrainedFootFrames =  new SideDependentList<>();
   private final SideDependentList<FramePose3D> newFootFramePoses = new SideDependentList<>();

   private final RDXVRSteppingTracker steppingTracker = new RDXVRSteppingTracker();
   private final SideDependentList<Boolean> isFootInContact = new SideDependentList<>();
   private boolean controllingFeet = false;

   private static final double PELVIS_ALPHA = 0.05;
   private final FramePose3D filteredPelvisFramePose = new FramePose3D(); // low-pass state

   /**
    * Constructor for the motion retargeting class.
    *
    * @param realRobotModel the real robot model
    * @param ghostRobotModel the KST robot model
    * @param controllerReferenceFrames the reference frames of the controllers
    * @param trackerReferenceFrames the reference frames of the trackers
    * @param retargetingParameters the retargeting parameters
    */
   public RDXVRMotionRetargeting(FullHumanoidRobotModel realRobotModel,
                                 FullHumanoidRobotModel ghostRobotModel,
                                 SideDependentList<MutableReferenceFrame> controllerReferenceFrames,
                                 Map<String, MutableReferenceFrame> trackerReferenceFrames,
                                 MutableReferenceFrame headsetReferenceFrame,
                                 RetargetingParameters retargetingParameters)
   {
      this.realRobotModel = realRobotModel;
      this.ghostRobotModel = ghostRobotModel;
      this.retargetingParameters = retargetingParameters;
      this.controllerReferenceFrames = controllerReferenceFrames;
      this.trackerReferenceFrames = trackerReferenceFrames;
      this.headsetReferenceFrame = headsetReferenceFrame;

      for (RobotSide side : RobotSide.values)
      {
         shoulderFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         scaledHandsFrameGraphics.put(side, new RDXReferenceFrameGraphic(FRAME_AXIS_GRAPHICS_LENGTH));
         shoulderToScaledHandTransforms.put(side, new RigidBodyTransform());
         initialFootTrackerTransformsToWorld.put(side, new RigidBodyTransform());
         initialFootTransformsToWorld.put(side, new RigidBodyTransform());
         newFootFramePoses.put(side, new FramePose3D());
         isFootInContact.put(side, true);
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
      retargetFeet();
   }

   /**
    * Compute the desired pose of pelvis.
    * Offset the pelvis control frame based on the initial user/robot height mismatch.
    *
    */
   private void retargetPelvis()
   {
      if (trackerReferenceFrames.containsKey(WAIST.getSegmentName()) && !controlArmsOnly)
      {
         if (initialPelvisFrame == null)
         {
            RigidBodyTransform initialWaistTrackerTransform = trackerReferenceFrames.get(WAIST.getSegmentName())
                                                                                    .getReferenceFrame()
                                                                                    .getTransformToWorldFrame();
            initialPelvisTransformToWorld.set(realRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame());
            initialPelvisFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                          initialPelvisTransformToWorld);

            constrainedPelvisFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                            filteredPelvisFramePose);

            Vector3D offset = new Vector3D();
            offset.sub(initialWaistTrackerTransform.getTranslation(), initialPelvisTransformToWorld.getTranslation());
            ikControlFrameOffsetPosition.put(WAIST, new Point3D(offset));

            // Initialize filtered pose at current pelvis
            filteredPelvisFramePose.set(initialPelvisTransformToWorld);
         }

         // Calculate the variation of the tracker's frame from its initial value
         RigidBodyTransform waistTrackerTransform = new RigidBodyTransform(trackerReferenceFrames.get(WAIST.getSegmentName())
                                                                                                 .getReferenceFrame()
                                                                                                 .getTransformToWorldFrame());

         newPelvisFramePose.set(waistTrackerTransform);

         // Zero roll and pitch orientation variation as it can lead to very unnatural motions
         newPelvisFramePose.changeFrame(initialPelvisFrame);
         newPelvisFramePose.getRotation().setYawPitchRoll(0.0, 0.0, 0.0);
         newPelvisFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         double midFeetYaw = 0.5 * (ghostRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getRotation().getYaw()
                                    + ghostRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame().getRotation().getYaw());
         newPelvisFramePose.getRotation().setYawPitchRoll(midFeetYaw,
                                                          newPelvisFramePose.getRotation().getPitch(),
                                                          newPelvisFramePose.getRotation().getRoll());

         // --- Low-pass filter pelvis pose (position + orientation) ---
         // Position EMA, but DO NOT filter Z
         var filteredPos = filteredPelvisFramePose.getPosition();
         var newPos      = newPelvisFramePose.getPosition();

         // Filter X/Y
         filteredPos.setX((1.0 - PELVIS_ALPHA) * filteredPos.getX() + PELVIS_ALPHA * newPos.getX());
         filteredPos.setY((1.0 - PELVIS_ALPHA) * filteredPos.getY() + PELVIS_ALPHA * newPos.getY());
         // Pass-through Z (no interpolation)
         filteredPos.setZ(newPos.getZ());

         // Orientation EMA (slerp)
         filteredPelvisFramePose.getOrientation().interpolate(filteredPelvisFramePose.getOrientation(),
                                                              newPelvisFramePose.getOrientation(),
                                                              PELVIS_ALPHA);
         // ------------------------------------------------------------


         constrainedPelvisFrame.update();
         retargetedFrames.put(WAIST, constrainedPelvisFrame);
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
            Point3D leftFootXYInWorld = null;
            Point3D rightFootXYInWorld = null;
            if (centerOfMassDesiredXYInWorld == null)
            {
               Point3D leftSole = new Point3D(realRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getTranslation());
               Point3D rightSole = new Point3D(realRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame().getTranslation());
               // Compute the midpoint
               double midX = 0.5 * (leftSole.getX() + rightSole.getX());
               double midY = 0.5 * (leftSole.getY() + rightSole.getY());
               // Set the COM to the midpoint between the two soles
               centerOfMassDesiredXYInWorld = new Point3D(midX, midY, 0.0);

               leftFootXYInWorld = new Point3D(realRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame().getTranslation());
               rightFootXYInWorld = new Point3D(realRobotModel.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame().getTranslation());
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
            if (normalizedOffset >= 1.0)
               normalizedOffset = 1.0;
            else if (normalizedOffset <= 0.0)
            {
               normalizedOffset = 0.0;
            }
            // Apply a low-pass filter to smooth normalized offset fluctuations
            double alpha = 0.15; // tuning parameter: smaller = smoother, larger = more responsive
            double filteredNormalizedOffset = previousOffsetValue + alpha * (normalizedOffset - previousOffsetValue);
            // Clamp to [0,1]
            filteredNormalizedOffset = Math.max(0.0, Math.min(1.0, filteredNormalizedOffset));
            // Store for next iteration
            previousOffsetValue = filteredNormalizedOffset;

            if (leftFootXYInWorld == null || rightFootXYInWorld == null)
            {
               leftFootXYInWorld = new Point3D(leftAnkleTrackerXYInWorld);
               rightFootXYInWorld = new Point3D(rightAnkleTrackerXYInWorld);
            }

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

   /**
    * Compute the desired pose of the feet.
    * Offset the feet control frames based the initial user/robot position mismatch.
    */
   private void retargetFeet()
   {
      if (trackerReferenceFrames.containsKey(LEFT_ANKLE.getSegmentName())
              && trackerReferenceFrames.containsKey(RIGHT_ANKLE.getSegmentName())
              && !controlArmsOnly)
      {
         controllingFeet = true;
         for (RobotSide side : RobotSide.values())
         {
            VRTrackedSegmentType footSegment = side == RobotSide.LEFT ? LEFT_ANKLE : RIGHT_ANKLE;
            String footName = footSegment.getSegmentName();
            if (initialFootFrames.get(side) == null)
            {
               ReferenceFrame trackerFootFrame = trackerReferenceFrames.get(footName).getReferenceFrame();
               initialFootTrackerTransformsToWorld.get(side).set(trackerFootFrame.getTransformToWorldFrame());

               initialFootTransformsToWorld.get(side).set(realRobotModel.getSoleFrame(side).getTransformToWorldFrame());
               initialFootFrames.put(side, ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                    initialFootTransformsToWorld.get(side)));

               constrainedFootFrames.put(side, ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), newFootFramePoses.get(side)));

               Vector3D offset = new Vector3D();
               offset.sub(initialFootTrackerTransformsToWorld.get(side).getTranslation(), initialFootTransformsToWorld.get(side).getTranslation());
               ikControlFrameOffsetPosition.put(footSegment, new Point3D(offset));
            }
            steppingTracker.processVRInput(side, trackerReferenceFrames.get(footName).getReferenceFrame().getTransformToWorldFrame());

            // Concatenate the initial foot transform with the variation
            RigidBodyTransform footTrackerTransform = new RigidBodyTransform(trackerReferenceFrames.get(footName)
                                                                                                   .getReferenceFrame()
                                                                                                   .getTransformToWorldFrame());
            newFootFramePoses.get(side).set(footTrackerTransform);

            // Build a constrained pose with yaw-only orientation in the initial foot frame
            FramePose3D constrainedFootPose = new FramePose3D(newFootFramePoses.get(side));
            constrainedFootPose.changeFrame(initialFootFrames.get(side));
            constrainedFootPose.getRotation().setYawPitchRoll(newFootFramePoses.get(side).getRotation().getYaw(), 0.0,0.0);
            constrainedFootPose.changeFrame(ReferenceFrame.getWorldFrame());

            double alpha = steppingTracker.getLandingBlendFactor(side);
            if (alpha > 0.0)
            {
               Quaternion fullQ = new Quaternion();
               Quaternion constrainedQ = new Quaternion();
               Quaternion blendedQ = new Quaternion();

               fullQ.set(newFootFramePoses.get(side).getOrientation());
               constrainedQ.set(constrainedFootPose.getOrientation());

               // alpha=0 -> fullQ, alpha=1 -> constrainedQ
               blendedQ.interpolate(constrainedQ, fullQ, 1.0 - alpha);

               newFootFramePoses.get(side).getOrientation().set(blendedQ);
            }
            constrainedFootFrames.get(side).update();

            isFootInContact.put(side, steppingTracker.isFootInContact(side));
            if (!steppingTracker.isFootInContact(side))
            {
               retargetedFrames.put(side == RobotSide.LEFT ? LEFT_ANKLE : RIGHT_ANKLE, constrainedFootFrames.get(side));
            }
         }
      }
      else
      {
         controllingFeet = false;
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
      {
         initialHandPositionsInWorld.put(side, null);
         initialFootFrames.put(side, null);
         isFootInContact.put(side, true);
      }
      retargetedFrames.clear();
      centerOfMassDesiredXYInWorld = null;
      previousOffsetValue = 0.5;
      controllingFeet = false;
      steppingTracker.reset();
      desiredLinearVelocities.clear();
      desiredAngularVelocities.clear();
   }

   public Set<VRTrackedSegmentType> getRetargetedSegments()
   {
      return retargetedFrames.keySet();
   }

   public void setDesiredVelocities(VRTrackedSegmentType segment, FrameVector3D desiredAngularVelocity, FrameVector3D desiredLinearVelocity)
   {
      desiredAngularVelocities.put(segment, desiredAngularVelocity);
      desiredLinearVelocities.put(segment, desiredLinearVelocity);
   }

   public ReferenceFrame getDesiredFrame(VRTrackedSegmentType segment)
   {
      return retargetedFrames.get(segment);
   }

   public FrameVector3D getDesiredLinearVelocity(VRTrackedSegmentType segment)
   {
      return desiredLinearVelocities.get(segment);
   }

   public FrameVector3D getDesiredAngularVelocity(VRTrackedSegmentType segment)
   {
      return desiredAngularVelocities.get(segment);
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

   public boolean isFootInContact(RobotSide side)
   {
      return isFootInContact.get(side);
   }

   public boolean isControllingFeet()
   {
      return controllingFeet;
   }

   public Point3D getIKControlFramePosition(VRTrackedSegmentType segmentType)
   {
      return ikControlFrameOffsetPosition.get(segmentType);
   }
}
