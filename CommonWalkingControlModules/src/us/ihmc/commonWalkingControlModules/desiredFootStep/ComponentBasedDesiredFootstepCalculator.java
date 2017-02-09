package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ComponentBasedDesiredFootstepCalculator extends AbstractDesiredFootstepCalculator
{
   private final BooleanYoVariable matchSupportFootPlane = new BooleanYoVariable("matchSupportFootPlane", registry);

   private final DoubleYoVariable inPlaceWidth = new DoubleYoVariable("inPlaceWidth", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable minStepWidth = new DoubleYoVariable("minStepWidth", registry);
   private final DoubleYoVariable maxStepWidth = new DoubleYoVariable("maxStepWidth", registry);

   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);

   private final DoubleYoVariable velocityMagnitudeInHeading = new DoubleYoVariable("velocityMagnitudeInHeading", registry);
   private final DoubleYoVariable velocityMagnitudeToLeftOfHeading = new DoubleYoVariable("velocityMagnitudeToLeftOfHeading", registry);

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ZUpFrame> soleZUpFrames = new SideDependentList<>();

   private final DesiredVelocityControlModule desiredVelocityControlModule;

   private HeightMap heightMap;

   private final FrameOrientation2d pelvisOrientation2d = new FrameOrientation2d();
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
         pelvisOrientation2d.set(pelvisZUpFrame);
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
      Matrix3d footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame);
      FramePoint footstepPosition = getDesiredFootstepPosition(supportZUpFrame, swingLegSide, footToWorldRotation, 0.0, stepDuration);

      setYoVariables(swingLegSide, footToWorldRotation, footstepPosition.getVectorCopy());
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
      Matrix3d footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame);
      FrameOrientation footstepOrientation = new FrameOrientation(worldFrame, footToWorldRotation);
      FramePoint footstepPosition = getDesiredFootstepPosition(futureSupportZUpFrame, futureSwingLegSide, footToWorldRotation, timeFromNow, stepDuration);
      footstepPosition.changeFrame(worldFrame);

      FootstepDataMessage predictedFootstep = new FootstepDataMessage();
      predictedFootstep.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      predictedFootstep.setRobotSide(futureSwingLegSide);
      predictedFootstep.setLocation(footstepPosition.getPoint());
      predictedFootstep.setOrientation(footstepOrientation.getQuaternion());
      return predictedFootstep;
   }

   private FramePoint getDesiredFootstepPosition(ReferenceFrame supportZUpFrame, RobotSide swingLegSide, Matrix3d footToWorldRotation, double timeFromNow,
         double stepDuration)
   {
      FrameVector2d desiredOffsetFromAnkle = computeDesiredOffsetFromSupport(swingLegSide, timeFromNow, stepDuration);
      FramePoint footstepPosition = computeDesiredFootPosition(swingLegSide, supportZUpFrame, desiredOffsetFromAnkle, footToWorldRotation);
      footstepPosition.changeFrame(worldFrame);

      return footstepPosition;
   }

   private final FrameVector2d desiredHeading = new FrameVector2d();
   private final FrameVector2d desiredVelocity = new FrameVector2d();
   private final FrameVector2d toLeftOfDesiredHeading = new FrameVector2d();

   // TODO: clean up
   private FrameVector2d computeDesiredOffsetFromSupport(RobotSide swingLegSide, double timeFromNow, double stepDuration)
   {
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
      ReferenceFrame predictedHeadingFrame = desiredHeadingControlModule.getPredictedHeadingFrame(timeFromNow);
      desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      desiredHeadingControlModule.getDesiredHeading(desiredHeading, timeFromNow);

      toLeftOfDesiredHeading.setIncludingFrame(desiredHeading.getReferenceFrame(), -desiredHeading.getY(), desiredHeading.getX());
      desiredVelocity.changeFrame(desiredHeading.getReferenceFrame());
      velocityMagnitudeInHeading.set(desiredVelocity.dot(desiredHeading));
      velocityMagnitudeToLeftOfHeading.set(desiredVelocity.dot(toLeftOfDesiredHeading));

      FrameVector2d desiredOffsetInPredictedHeadingFrame = new FrameVector2d();
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

      FrameVector2d desiredOffsetFromSupport = new FrameVector2d(desiredOffsetInPredictedHeadingFrame);
      desiredOffsetFromSupport.add(0.0, swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue()));

      if (swingLegSide == RobotSide.LEFT)
      {
         desiredOffsetFromSupport.setY(MathTools.clipToMinMax(desiredOffsetFromSupport.getY(), minStepWidth.getDoubleValue(), maxStepWidth.getDoubleValue()));
      }
      else
      {
         desiredOffsetFromSupport.setY(MathTools.clipToMinMax(desiredOffsetFromSupport.getY(), -maxStepWidth.getDoubleValue(), -minStepWidth.getDoubleValue()));
      }

      double stepLength = desiredOffsetFromSupport.length();
      if (stepLength > maxStepLength.getDoubleValue())
      {
         desiredOffsetFromSupport.scale(maxStepLength.getDoubleValue() / stepLength);
      }

      return desiredOffsetFromSupport;
   }

   private Matrix3d computeDesiredFootRotation(ReferenceFrame desiredHeadingFrame)
   {
      RigidBodyTransform footToSupportTransform = desiredHeadingFrame.getTransformToDesiredFrame(worldFrame);
      Matrix3d footToSupportRotation = new Matrix3d();
      footToSupportTransform.getRotation(footToSupportRotation);
      double yaw = RotationTools.computeYaw(footToSupportRotation);
      double pitch = stepPitch.getDoubleValue();
      double roll = 0.0;
      RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, footToSupportRotation);

      return footToSupportRotation;
   }

   private FramePoint computeDesiredFootPosition(RobotSide upcomingSwingLegSide, ReferenceFrame upcomingSupportZUpFrame, FrameVector2d desiredOffsetFromSupport,
         Matrix3d swingFootToWorldRotation)
   {
      ContactablePlaneBody upcomingSwingFoot = contactableBodies.get(upcomingSwingLegSide);
      desiredOffsetFromSupport.changeFrame(upcomingSupportZUpFrame);
      FramePoint footstepPosition = new FramePoint(upcomingSupportZUpFrame, desiredOffsetFromSupport.getX(), desiredOffsetFromSupport.getY(), 0.0);
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

         FramePoint upcomingSwingAnkle = new FramePoint(upcomingSwingFoot.getSoleFrame());
         upcomingSwingAnkle.changeFrame(worldFrame);
         double zUpcomingSwing = upcomingSwingAnkle.getZ();

         FrameVector searchDirection = new FrameVector(upcomingSupportZUpFrame, 0.0, 0.0, -1.0);
         List<FramePoint> contactPointsCopy = upcomingSwingFoot.getContactPointsCopy();
         FramePoint upcomingSwingMinZPoint = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactPointsCopy, searchDirection, 1).get(0);
         FrameVector deltaZ = new FrameVector(upcomingSwingMinZPoint);
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

   @Override
   public boolean isDone()
   {
      return false;
   }
}
