package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.PoseReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;


public class ComponentBasedDesiredFootstepCalculator extends AbstractAdjustableDesiredFootstepCalculator
{
   private final BooleanYoVariable matchSupportFootPlane = new BooleanYoVariable("matchSupportFootPlane", registry);

   private final DoubleYoVariable inPlaceWidth = new DoubleYoVariable("inPlaceWidth", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable minStepWidth = new DoubleYoVariable("minStepWidth", registry);
   private final DoubleYoVariable maxStepWidth = new DoubleYoVariable("maxStepWidth", registry);

   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);

   private final DoubleYoVariable velocityMagnitudeInHeading = new DoubleYoVariable("velocityMagnitudeInHeading", registry);
   private final DoubleYoVariable velocityMagnitudeToLeftOfHeading = new DoubleYoVariable("velocityMagnitudeToLeftOfHeading", registry);

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<? extends ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<? extends ReferenceFrame> ankleFrames;

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;

   private HeightMap heightMap;

   private final double ankleHeight;

   private final FrameOrientation2d pelvisOrientation2d = new FrameOrientation2d();

   public ComponentBasedDesiredFootstepCalculator(double ankleHeight, ReferenceFrame pelvisZUpFrame, SideDependentList<? extends ReferenceFrame> ankleZUpFrames,
         SideDependentList<? extends ReferenceFrame> ankleFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
         DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry)
   {
      super(bipedFeet, getFramesToStoreFootstepsIn(), parentRegistry);

      this.ankleHeight = ankleHeight;
      this.pelvisZUpFrame = pelvisZUpFrame;
      this.ankleZUpFrames = ankleZUpFrames;
      this.ankleFrames = ankleFrames;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;

      matchSupportFootPlane.set(false);
   }

   @Override
   public void initialize()
   {
      if (pelvisZUpFrame != null)
      {
         pelvisOrientation2d.set(pelvisZUpFrame);
         pelvisOrientation2d.changeFrame(ReferenceFrame.getWorldFrame());
         desiredHeadingControlModule.resetHeadingAngle(pelvisOrientation2d.getYaw());
      }
   }

   public void setGroundProfile(HeightMap heightMap)
   {
      this.heightMap = heightMap;
   }

   @Override
   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();
      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportLegSide);
      ReferenceFrame supportAnkleFrame = ankleFrames.get(supportLegSide);

      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      Matrix3d footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame, supportAnkleFrame);

      FramePoint footstepPosition = getDesiredFootstepPosition(supportAnkleZUpFrame, supportAnkleFrame, swingLegSide, desiredHeadingFrame, footToWorldRotation);

      setYoVariables(swingLegSide, footToWorldRotation, footstepPosition.getVectorCopy());
   }

   @Override
   public Footstep predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, Footstep desiredFootstep)
   {
      RobotSide futureSwingLegSide = supportLegSide;
      ReferenceFrame futureSupportAnkleZUpFrame = desiredFootstep.getPoseReferenceFrame();
      ReferenceFrame futureSupportAnkleFrame = desiredFootstep.getPoseReferenceFrame();

      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      Matrix3d footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame, futureSupportAnkleFrame);

      FramePoint footstepPosition = getDesiredFootstepPosition(futureSupportAnkleZUpFrame, futureSupportAnkleFrame, futureSwingLegSide, desiredHeadingFrame,
            footToWorldRotation);
      FrameOrientation footstepOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
      double[] yawPitchRoll = new double[3];
      RotationFunctions.getYawPitchRoll(yawPitchRoll, footToWorldRotation);
      footstepOrientation.setYawPitchRoll(yawPitchRoll);

      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", footstepPose);

      ContactablePlaneBody foot = contactableBodies.get(futureSwingLegSide);
      boolean trustHeight = true;
      return new Footstep(foot.getRigidBody(), futureSwingLegSide, foot.getSoleFrame(), poseReferenceFrame, trustHeight);
   }

   private FramePoint getDesiredFootstepPosition(ReferenceFrame supportAnkleZUpFrame, ReferenceFrame supportAnkleFrame, RobotSide swingLegSide,
         ReferenceFrame desiredHeadingFrame, Matrix3d footToWorldRotation)
   {
      FrameVector2d desiredOffsetFromAnkle = computeDesiredOffsetFromSupportAnkle(swingLegSide, desiredHeadingFrame);
      FramePoint footstepPosition = computeDesiredFootPosition(swingLegSide, supportAnkleZUpFrame, supportAnkleFrame, desiredOffsetFromAnkle,
            footToWorldRotation);
      footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());

      return footstepPosition;
   }

   private final FrameVector2d desiredHeading = new FrameVector2d();
   private final FrameVector2d desiredVelocity = new FrameVector2d();
   private final FrameVector2d toLeftOfDesiredHeading = new FrameVector2d();
   
   // TODO: clean up
   private FrameVector2d computeDesiredOffsetFromSupportAnkle(RobotSide swingLegSide, ReferenceFrame desiredHeadingFrame)
   {
      desiredHeadingControlModule.getDesiredHeading(desiredHeading);
      desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      toLeftOfDesiredHeading.setIncludingFrame(desiredHeading.getReferenceFrame(), -desiredHeading.getY(), desiredHeading.getX());

      desiredVelocity.changeFrame(desiredHeading.getReferenceFrame());
      velocityMagnitudeInHeading.set(desiredVelocity.dot(desiredHeading));
      velocityMagnitudeToLeftOfHeading.set(desiredVelocity.dot(toLeftOfDesiredHeading));

      FrameVector2d desiredVelocityInHeadingFrame = new FrameVector2d(desiredVelocity);
      desiredVelocityInHeadingFrame.changeFrame(desiredHeadingFrame);

      FrameVector2d desiredOffsetFromAnkle = new FrameVector2d(desiredHeadingFrame, 0.0, swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue())); // desiredVelocityInHeadingFrame);
      desiredOffsetFromAnkle.add(desiredVelocityInHeadingFrame);

      if (swingLegSide == RobotSide.LEFT)
      {
         desiredOffsetFromAnkle.setY(MathTools.clipToMinMax(desiredOffsetFromAnkle.getY(), minStepWidth.getDoubleValue(), maxStepWidth.getDoubleValue()));
      }
      else
      {
         desiredOffsetFromAnkle.setY(MathTools.clipToMinMax(desiredOffsetFromAnkle.getY(), -maxStepWidth.getDoubleValue(), -minStepWidth.getDoubleValue()));
      }

      double stepLength = desiredOffsetFromAnkle.length();
      if (stepLength > maxStepLength.getDoubleValue())
      {
         desiredOffsetFromAnkle.scale(maxStepLength.getDoubleValue() / stepLength);
      }

      return desiredOffsetFromAnkle;
   }

   private Matrix3d computeDesiredFootRotation(ReferenceFrame desiredHeadingFrame, ReferenceFrame supportFootFrame)
   {
      if (matchSupportFootPlane.getBooleanValue())
      {
         return computeDesiredFootRotationMatchSupportFootPlane(desiredHeadingFrame, supportFootFrame);
      }
      else
      {
         return computeDesiredFootRotationJustMatchSupportFootYaw(desiredHeadingFrame);
      }
   }

   private Matrix3d computeDesiredFootRotationMatchSupportFootPlane(ReferenceFrame desiredHeadingFrame, ReferenceFrame supportFootFrame)
   {
      RigidBodyTransform supportFootToWorldTransform = supportFootFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Matrix3d supportFootToWorldRotation = new Matrix3d();
      supportFootToWorldTransform.get(supportFootToWorldRotation);

      return supportFootToWorldRotation;
   }

   private Matrix3d computeDesiredFootRotationJustMatchSupportFootYaw(ReferenceFrame desiredHeadingFrame)
   {
      RigidBodyTransform footToSupportTransform = desiredHeadingFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Matrix3d footToSupportRotation = new Matrix3d();
      footToSupportTransform.get(footToSupportRotation);
      double yaw = RotationFunctions.getYaw(footToSupportRotation);
      double pitch = stepPitch.getDoubleValue();
      double roll = 0.0;
      RotationFunctions.setYawPitchRoll(footToSupportRotation, yaw, pitch, roll);

      return footToSupportRotation;
   }

   private FramePoint computeDesiredFootPosition(RobotSide upcomingSwingLegSide, ReferenceFrame upcomingSupportAnkleZUpFrame,
         ReferenceFrame upcomingSupportAnkleFrame, FrameVector2d desiredOffsetFromAnkle, Matrix3d swingFootToWorldRotation)
   {
      if (matchSupportFootPlane.getBooleanValue())
      {
         return computeDesiredFootPositionMatchSupportFootPlane(upcomingSwingLegSide, upcomingSupportAnkleFrame, desiredOffsetFromAnkle,
               swingFootToWorldRotation);
      }
      else
      {
         return computeDesiredFootPositionMatchMinimumZ(upcomingSwingLegSide, upcomingSupportAnkleZUpFrame, desiredOffsetFromAnkle, swingFootToWorldRotation);
      }
   }

   private FramePoint computeDesiredFootPositionMatchSupportFootPlane(RobotSide upcomingSwingLegSide, ReferenceFrame upcomingSupportAnkleFrame,
         FrameVector2d desiredOffsetFromAnkle, Matrix3d swingFootToWorldRotation)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      desiredOffsetFromAnkle.changeFrame(upcomingSupportAnkleFrame);
      FramePoint footstepPosition = new FramePoint(upcomingSupportAnkleFrame, desiredOffsetFromAnkle.getX(), desiredOffsetFromAnkle.getY(), 0.0);
      footstepPosition.changeFrame(worldFrame);

      return footstepPosition;
   }

   private FramePoint computeDesiredFootPositionMatchMinimumZ(RobotSide upcomingSwingLegSide, ReferenceFrame upcomingSupportAnkleZUpFrame,
         FrameVector2d desiredOffsetFromAnkle, Matrix3d swingFootToWorldRotation)
   {
      ContactablePlaneBody upcomingSwingFoot = contactableBodies.get(upcomingSwingLegSide);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      desiredOffsetFromAnkle.changeFrame(upcomingSupportAnkleZUpFrame);
      FramePoint footstepPosition = new FramePoint(upcomingSupportAnkleZUpFrame, desiredOffsetFromAnkle.getX(), desiredOffsetFromAnkle.getY(), 0.0);
      footstepPosition.changeFrame(worldFrame);

      double footstepMinZ = DesiredFootstepCalculatorTools.computeMinZPointWithRespectToAnkleInWorldFrame(swingFootToWorldRotation, upcomingSwingFoot);

      if (heightMap == null)
      {
         /*
          * Assume that the ground height is constant.
          *
          * Specifically, if we assume that: 1) the lowest contact point on the
          * upcoming swing foot is in contact with the ground 2) the ground
          * height at the lowest upcoming swing foot contact point is the same
          * as the ground height at the lowest swing foot contact point
          *
          * then the following holds:
          *
          * let upcomingSwingMinZ be the z coordinate of the vector (expressed
          * in world frame) from upcoming swing ankle to the lowest contact
          * point on the stance foot (compared in world frame). Current foot
          * orientation is used to determine this value.
          *
          * let footstepMinZ be the z coordinate of the vector (expressed in
          * world frame) from planned swing ankle to the lowest contact point on
          * the planned swing foot (compared in world frame). Planned foot
          * orientation is used to determine this value
          *
          * let zUpcomingSwing be the z coordinate of the upcoming swing ankle,
          * expressed in world frame. let zFootstep be the z coordinate of the
          * footstep, expressed in world frame (this is what we're after) let
          * zGround be the z coordinate of the lowest point on the stance foot
          * i.e. of the ground zUpcomingSwing = zGround - upcomingSwingMinZ
          * zFootstep = zGround - footstepMinZ = zUpcomingSwing +
          * upcomingSwingMinZ - footstepMinZ
          */

         FramePoint upcomingSwingAnkle = new FramePoint(upcomingSwingFoot.getFrameAfterParentJoint());
         upcomingSwingAnkle.changeFrame(worldFrame);
         double zUpcomingSwing = upcomingSwingAnkle.getZ();

         FrameVector searchDirection = new FrameVector(upcomingSupportAnkleZUpFrame, 0.0, 0.0, -1.0);
         FramePoint upcomingSwingMinZPoint = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(upcomingSwingFoot.getContactPointsCopy(),
               searchDirection, 1).get(0);
         upcomingSwingMinZPoint.changeFrame(ankleZUpFrames.get(upcomingSwingLegSide));
         double upcomingSwingMinZ = upcomingSwingMinZPoint.getZ();

         double zFootstep = zUpcomingSwing + upcomingSwingMinZ - footstepMinZ;
         footstepPosition.setZ(zFootstep);
      }
      else
      {
         /*
          * use ground profile to determine height at the planned foot position
          */
         footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());
         double groundZ = heightMap.heightAt(footstepPosition.getX(), footstepPosition.getY(), 0.0);
         double ankleZ = groundZ + ankleHeight;

         footstepPosition.setZ(ankleZ);
      }

      return footstepPosition;
   }

   private void setYoVariables(RobotSide swingLegSide, Matrix3d rotation, Vector3d translation)
   {
      footstepOrientations.get(swingLegSide).set(rotation);
      footstepPositions.get(swingLegSide).set(translation);
   }

   public void setInPlaceWidth(double inPlaceWidth)
   {
      this.inPlaceWidth.set(inPlaceWidth);
   }

   public void setMaxStepLength(double maxStepLength)
   {
      this.maxStepLength.set(maxStepLength);
   }

   public void setMinStepWidth(double minStepWidth)
   {
      this.minStepWidth.set(minStepWidth);
   }

   public void setMaxStepWidth(double maxStepWidth)
   {
      this.maxStepWidth.set(maxStepWidth);
   }

   public void setStepPitch(double stepPitch)
   {
      this.stepPitch.set(stepPitch);
   }

   private static SideDependentList<ReferenceFrame> getFramesToStoreFootstepsIn()
   {
      return new SideDependentList<ReferenceFrame>(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   @Override
   protected List<FramePoint> getContactPoints(RobotSide swingSide)
   {
      double stepPitch = this.stepPitch.getDoubleValue();
      List<FramePoint> allContactPoints = contactableBodies.get(swingSide).getContactPointsCopy();
      if (stepPitch == 0.0)
      {
         return allContactPoints;
      }
      else
      {
         FrameVector forwardInFootFrame = new FrameVector(contactableBodies.get(swingSide).getFrameAfterParentJoint());
         ReferenceFrame frame = allContactPoints.get(0).getReferenceFrame();
         forwardInFootFrame.changeFrame(frame);
         forwardInFootFrame.scale(Math.signum(stepPitch));
         int nPoints = 2;

         return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(allContactPoints, forwardInFootFrame, nPoints);
      }
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
