package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ComponentBasedDesiredFootstepCalculator extends AbstractDesiredFootstepCalculator
{
   private final YoBoolean matchSupportFootPlane = new YoBoolean("matchSupportFootPlane", registry);

   private final YoDouble inPlaceWidth = new YoDouble("inPlaceWidth", registry);
   private final YoDouble maxStepLength = new YoDouble("maxStepLength", registry);

   private final YoDouble minStepWidth = new YoDouble("minStepWidth", registry);
   private final YoDouble maxStepWidth = new YoDouble("maxStepWidth", registry);

   private final YoDouble stepPitch = new YoDouble("stepPitch", registry);

   private final YoDouble velocityMagnitudeInHeading = new YoDouble("velocityMagnitudeInHeading", registry);
   private final YoDouble velocityMagnitudeToLeftOfHeading = new YoDouble("velocityMagnitudeToLeftOfHeading", registry);

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ZUpFrame> soleZUpFrames = new SideDependentList<>();

   private final DesiredVelocityControlModule desiredVelocityControlModule;

   private HeightMap heightMap;

   private final FrameOrientation2D pelvisOrientation2d = new FrameOrientation2D();
   private final SideDependentList<? extends ContactablePlaneBody> contactableBodies;

   public ComponentBasedDesiredFootstepCalculator(ReferenceFrame pelvisZUpFrame, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
         DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry)
   {
      super(parentRegistry);

      this.pelvisZUpFrame = pelvisZUpFrame;
      this.contactableBodies = bipedFeet;
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = contactableBodies.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
         soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, soleFrame, "soleZUpFrame"));
      }

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;

      matchSupportFootPlane.set(false);
   }

   @Override
   public void initialize()
   {
      if (pelvisZUpFrame != null)
      {
         pelvisOrientation2d.setToZero(pelvisZUpFrame);
         pelvisOrientation2d.changeFrame(worldFrame);
         desiredHeadingControlModule.resetHeadingAngle(pelvisOrientation2d.getYaw());
      }
   }

   public void setGroundProfile(HeightMap heightMap)
   {
      this.heightMap = heightMap;
   }

   @Override
   public void initializeDesiredFootstep(RobotSide supportLegSide, double stepDuration)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();
      ReferenceFrame supportZUpFrame = soleZUpFrames.get(supportLegSide);
      supportZUpFrame.update();

      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      RotationMatrix footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame);
      FramePoint3D footstepPosition = getDesiredFootstepPosition(supportZUpFrame, swingLegSide, footToWorldRotation, 0.0, stepDuration);

      setYoVariables(swingLegSide, footToWorldRotation, new Vector3D(footstepPosition));
   }

   @Override
   public FootstepDataMessage predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, FootstepDataMessage desiredFootstep, double timeFromNow,
         double stepDuration)
   {
      RobotSide futureSwingLegSide = supportLegSide;
      PoseReferenceFrame futureSupportFrame = new PoseReferenceFrame("futureSupportFrame", worldFrame);
      futureSupportFrame.setPoseAndUpdate(desiredFootstep.getLocation(), desiredFootstep.getOrientation());
      ZUpFrame futureSupportZUpFrame = new ZUpFrame(worldFrame, futureSupportFrame, "futureSupportZUpFrame");
      futureSupportZUpFrame.update();

      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getPredictedHeadingFrame(timeFromNow);
      RotationMatrix footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame);
      FrameQuaternion footstepOrientation = new FrameQuaternion(worldFrame, footToWorldRotation);
      FramePoint3D footstepPosition = getDesiredFootstepPosition(futureSupportZUpFrame, futureSwingLegSide, footToWorldRotation, timeFromNow, stepDuration);
      footstepPosition.changeFrame(worldFrame);

      FootstepDataMessage predictedFootstep = new FootstepDataMessage();
      predictedFootstep.setRobotSide(futureSwingLegSide.toByte());
      predictedFootstep.setLocation(footstepPosition);
      predictedFootstep.setOrientation(footstepOrientation);
      return predictedFootstep;
   }

   private FramePoint3D getDesiredFootstepPosition(ReferenceFrame supportZUpFrame, RobotSide swingLegSide, RotationMatrix footToWorldRotation, double timeFromNow,
         double stepDuration)
   {
      FrameVector2D desiredOffsetFromAnkle = computeDesiredOffsetFromSupport(swingLegSide, timeFromNow, stepDuration);
      FramePoint3D footstepPosition = computeDesiredFootPosition(swingLegSide, supportZUpFrame, desiredOffsetFromAnkle, footToWorldRotation);
      footstepPosition.changeFrame(worldFrame);

      return footstepPosition;
   }

   private final FrameVector2D desiredHeading = new FrameVector2D();
   private final FrameVector2D desiredVelocity = new FrameVector2D();
   private final FrameVector2D toLeftOfDesiredHeading = new FrameVector2D();

   // TODO: clean up
   private FrameVector2D computeDesiredOffsetFromSupport(RobotSide swingLegSide, double timeFromNow, double stepDuration)
   {
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      ReferenceFrame predictedHeadingFrame = desiredHeadingControlModule.getPredictedHeadingFrame(timeFromNow);
      desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      desiredHeadingControlModule.getDesiredHeading(desiredHeading, timeFromNow);

      toLeftOfDesiredHeading.setIncludingFrame(desiredHeading.getReferenceFrame(), -desiredHeading.getY(), desiredHeading.getX());
      desiredVelocity.changeFrame(desiredHeading.getReferenceFrame());
      velocityMagnitudeInHeading.set(desiredVelocity.dot(desiredHeading));
      velocityMagnitudeToLeftOfHeading.set(desiredVelocity.dot(toLeftOfDesiredHeading));

      FrameVector2D desiredOffsetInPredictedHeadingFrame = new FrameVector2D();
      if (desiredVelocityControlModule.getReferenceFrame().equals(desiredHeadingFrame))
      {
         // Assume constant velocity in rotating heading frame.
         desiredVelocity.changeFrame(desiredHeadingFrame);
         desiredOffsetInPredictedHeadingFrame.changeFrame(predictedHeadingFrame);
         desiredOffsetInPredictedHeadingFrame.set(desiredVelocity.getX(), desiredVelocity.getY());
         desiredOffsetInPredictedHeadingFrame.scale(stepDuration);
      }
      else
      {
         // Assume constant velocity in fixed reference frame.
         desiredOffsetInPredictedHeadingFrame.setIncludingFrame(desiredVelocity);
         desiredOffsetInPredictedHeadingFrame.changeFrame(predictedHeadingFrame);
         desiredOffsetInPredictedHeadingFrame.scale(stepDuration);
      }

      FrameVector2D desiredOffsetFromSupport = new FrameVector2D(desiredOffsetInPredictedHeadingFrame);
      desiredOffsetFromSupport.add(0.0, swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue()));

      if (swingLegSide == RobotSide.LEFT)
      {
         desiredOffsetFromSupport.setY(MathTools.clamp(desiredOffsetFromSupport.getY(), minStepWidth.getDoubleValue(), maxStepWidth.getDoubleValue()));
      }
      else
      {
         desiredOffsetFromSupport.setY(MathTools.clamp(desiredOffsetFromSupport.getY(), -maxStepWidth.getDoubleValue(), -minStepWidth.getDoubleValue()));
      }

      double stepLength = desiredOffsetFromSupport.length();
      if (stepLength > maxStepLength.getDoubleValue())
      {
         desiredOffsetFromSupport.scale(maxStepLength.getDoubleValue() / stepLength);
      }

      return desiredOffsetFromSupport;
   }

   private RotationMatrix computeDesiredFootRotation(ReferenceFrame desiredHeadingFrame)
   {
      RigidBodyTransform footToSupportTransform = desiredHeadingFrame.getTransformToDesiredFrame(worldFrame);
      RotationMatrix footToSupportRotation = new RotationMatrix();
      footToSupportTransform.getRotation(footToSupportRotation);
      double yaw = footToSupportRotation.getYaw();
      double pitch = stepPitch.getDoubleValue();
      double roll = 0.0;
      footToSupportRotation.setYawPitchRoll(yaw, pitch, roll);

      return footToSupportRotation;
   }

   private FramePoint3D computeDesiredFootPosition(RobotSide upcomingSwingLegSide, ReferenceFrame upcomingSupportZUpFrame, FrameVector2D desiredOffsetFromSupport,
         RotationMatrix swingFootToWorldRotation)
   {
      ContactablePlaneBody upcomingSwingFoot = contactableBodies.get(upcomingSwingLegSide);
      desiredOffsetFromSupport.changeFrame(upcomingSupportZUpFrame);
      FramePoint3D footstepPosition = new FramePoint3D(upcomingSupportZUpFrame, desiredOffsetFromSupport.getX(), desiredOffsetFromSupport.getY(), 0.0);
      footstepPosition.changeFrame(worldFrame);

      double footstepMinZ = DesiredFootstepCalculatorTools.computeMinZPointWithRespectToSoleInWorldFrame(swingFootToWorldRotation, upcomingSwingFoot);

      if (heightMap == null)
      {
         /*
          * Assume that the ground height is constant. Specifically, if we
          * assume that: 1) the lowest contact point on the upcoming swing foot
          * is in contact with the ground 2) the ground height at the lowest
          * upcoming swing foot contact point is the same as the ground height
          * at the lowest swing foot contact point then the following holds: let
          * upcomingSwingMinZ be the z coordinate of the vector (expressed in
          * world frame) from upcoming swing ankle to the lowest contact point
          * on the stance foot (compared in world frame). Current foot
          * orientation is used to determine this value. let footstepMinZ be the
          * z coordinate of the vector (expressed in world frame) from planned
          * swing ankle to the lowest contact point on the planned swing foot
          * (compared in world frame). Planned foot orientation is used to
          * determine this value let zUpcomingSwing be the z coordinate of the
          * upcoming swing ankle, expressed in world frame. let zFootstep be the
          * z coordinate of the footstep, expressed in world frame (this is what
          * we're after) let zGround be the z coordinate of the lowest point on
          * the stance foot i.e. of the ground zUpcomingSwing = zGround -
          * upcomingSwingMinZ zFootstep = zGround - footstepMinZ =
          * zUpcomingSwing + upcomingSwingMinZ - footstepMinZ
          */

         FramePoint3D upcomingSwingAnkle = new FramePoint3D(upcomingSwingFoot.getSoleFrame());
         upcomingSwingAnkle.changeFrame(worldFrame);
         double zUpcomingSwing = upcomingSwingAnkle.getZ();

         FrameVector3D searchDirection = new FrameVector3D(upcomingSupportZUpFrame, 0.0, 0.0, -1.0);
         List<FramePoint3D> contactPointsCopy = upcomingSwingFoot.getContactPointsCopy();
         FramePoint3D upcomingSwingMinZPoint = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactPointsCopy, searchDirection, 1).get(0);
         FrameVector3D deltaZ = new FrameVector3D(upcomingSwingMinZPoint);
         deltaZ.changeFrame(worldFrame);
         double upcomingSwingMinZ = deltaZ.getZ();

         double zFootstep = zUpcomingSwing + upcomingSwingMinZ - footstepMinZ;
         footstepPosition.setZ(zFootstep);
      }
      else
      {
         /*
          * use ground profile to determine height at the planned foot position
          */
         footstepPosition.changeFrame(worldFrame);
         double groundZ = heightMap.heightAt(footstepPosition.getX(), footstepPosition.getY(), 0.0);

         footstepPosition.setZ(groundZ);
      }

      return footstepPosition;
   }

   private void setYoVariables(RobotSide swingLegSide, RotationMatrix rotation, Vector3D translation)
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

   @Override
   public boolean isDone()
   {
      return false;
   }
}
