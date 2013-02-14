package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredVelocityControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.GroundProfile;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComponentBasedDesiredFootstepCalculator extends AbstractAdjustableDesiredFootstepCalculator
{
   private final DoubleYoVariable inPlaceWidth = new DoubleYoVariable("inPlaceWidth", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable minStepWidth = new DoubleYoVariable("minStepWidth", registry);
   private final DoubleYoVariable maxStepWidth = new DoubleYoVariable("maxStepWidth", registry);

   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);

   private final DoubleYoVariable velocityMagnitudeInHeading = new DoubleYoVariable("velocityMagnitudeInHeading", registry);
   private final DoubleYoVariable velocityMagnitudeToLeftOfHeading = new DoubleYoVariable("velocityMagnitudeToLeftOfHeading", registry);

   private SideDependentList<? extends ReferenceFrame> ankleZUpFrames;

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;

   private GroundProfile groundProfile;

   public ComponentBasedDesiredFootstepCalculator(SideDependentList<? extends ReferenceFrame> ankleZUpFrames,
           SideDependentList<? extends ContactablePlaneBody> bipedFeet, DesiredHeadingControlModule desiredHeadingControlModule,
           DesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry)
   {
      super(bipedFeet, getFramesToStoreFootstepsIn(), parentRegistry);

      this.ankleZUpFrames = ankleZUpFrames;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
   }

   public void setGroundProfile(GroundProfile groundProfile)
   {
      this.groundProfile = groundProfile;
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportLegSide);
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      FrameVector2d desiredOffsetFromAnkle = computeDesiredOffsetFromSupportAnkle(swingLegSide, supportAnkleZUpFrame, desiredHeadingFrame);
      Matrix3d footToWorldRotation = computeDesiredFootRotation(desiredHeadingFrame);
      FramePoint footstepPosition = computeDesiredFootPosition(swingLegSide, supportAnkleZUpFrame, desiredOffsetFromAnkle, footToWorldRotation);
      footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());
      setYoVariables(swingLegSide, footToWorldRotation, footstepPosition.getVectorCopy());
   }

   // TODO: clean up
   private FrameVector2d computeDesiredOffsetFromSupportAnkle(RobotSide swingLegSide, ReferenceFrame supportAnkleZUpFrame, ReferenceFrame desiredHeadingFrame)
   {
      FrameVector2d desiredHeading = desiredHeadingControlModule.getDesiredHeading();
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      FrameVector2d toLeftOfDesiredHeading = new FrameVector2d(desiredHeading.getReferenceFrame(), -desiredHeading.getY(), desiredHeading.getX());

      desiredVelocity.changeFrame(desiredHeading.getReferenceFrame());
      velocityMagnitudeInHeading.set(desiredVelocity.dot(desiredHeading));
      velocityMagnitudeToLeftOfHeading.set(desiredVelocity.dot(toLeftOfDesiredHeading));

//    double stepForward = maxStepLength.getDoubleValue() * velocityMagnitudeInHeading.getDoubleValue();
//    double stepSideways = swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue());    // maxStepLength.getDoubleValue() * velocityMagnitudeToLeftOfHeading;

//    FrameVector2d desiredVelocityInSupportAnkleZUpFrame = desiredVelocity.changeFrameCopy(supportAnkleZUpFrame);
      FrameVector2d desiredVelocityInHeadingFrame = desiredVelocity.changeFrameCopy(desiredHeadingFrame);

      FrameVector2d desiredOffsetFromAnkle = new FrameVector2d(desiredHeadingFrame, 0.0, swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue()));    // desiredVelocityInHeadingFrame);
      desiredOffsetFromAnkle.add(desiredVelocityInHeadingFrame);

      if (desiredOffsetFromAnkle.getX() > maxStepLength.getDoubleValue())
         desiredOffsetFromAnkle.setX(maxStepLength.getDoubleValue());

      if (swingLegSide == RobotSide.LEFT)
      {
         desiredOffsetFromAnkle.setY(MathTools.clipToMinMax(desiredOffsetFromAnkle.getY(), minStepWidth.getDoubleValue(), maxStepWidth.getDoubleValue()));
      }
      else
      {
         desiredOffsetFromAnkle.setY(MathTools.clipToMinMax(desiredOffsetFromAnkle.getY(), -maxStepWidth.getDoubleValue(), -minStepWidth.getDoubleValue()));
      }

      return desiredOffsetFromAnkle;
   }

   private Matrix3d computeDesiredFootRotation(ReferenceFrame desiredHeadingFrame)
   {
      Transform3D footToSupportTransform = desiredHeadingFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Matrix3d footToSupportRotation = new Matrix3d();
      footToSupportTransform.get(footToSupportRotation);
      double yaw = RotationFunctions.getYaw(footToSupportRotation);
      double pitch = stepPitch.getDoubleValue();
      double roll = 0.0;
      RotationFunctions.setYawPitchRoll(footToSupportRotation, yaw, pitch, roll);

      return footToSupportRotation;
   }


   /**
    * @param swingLegSide
    * @param supportAnkleZUpFrame
    * @param desiredOffsetFromAnkle
    * @param swingFootToWorldRotation
    * @return
    */
   private FramePoint computeDesiredFootPosition(RobotSide swingLegSide, ReferenceFrame supportAnkleZUpFrame, FrameVector2d desiredOffsetFromAnkle,
           Matrix3d swingFootToWorldRotation)
   {
      RobotSide supportLegSide = swingLegSide.getOppositeSide();
      desiredOffsetFromAnkle.changeFrame(supportAnkleZUpFrame);

      FramePoint footstepPosition = new FramePoint(supportAnkleZUpFrame, desiredOffsetFromAnkle.getX(), desiredOffsetFromAnkle.getY(), 0.0);

      ContactablePlaneBody supportFoot = contactableBodies.get(supportLegSide);
      ContactablePlaneBody swingFoot = contactableBodies.get(swingLegSide);

      if (groundProfile == null)
      {
         /*
          * Assume that the ground height is constant.
          *
          * Specifically, if we assume that:
          * 1) the lowest contact point on the stance foot is in contact with the ground
          * 2) the ground height at the lowest stance foot contact point is the same as the ground height at the
          *    lowest swing foot contact point
          *
          * then the following holds:
          *
          * let stanceMinZ be the z coordinate of the vector (expressed in world frame) from stance ankle to the lowest contact point on
          * the stance foot (compared in world frame). Current foot orientation is used to determine this value.
          *
          * let swingMinZ be the z coordinate of the vector (expressed in world frame) from swing ankle to the lowest contact point on
          * the swing foot (compared in world frame). Planned foot orientation is used to determine this value
          *
          * let zStance be the z coordinate of the stance ankle, expressed in world frame.
          * let zSwing be the z coordinate of the swing ankle, expressed in world frame (this is what we're after)
          * let zGround be the z coordinate of the lowest point on the stance foot i.e. of the ground
          * zStance = zGround - stanceMinZ
          * zSwing = zGround - swingMinZ
          *        = zStance + stanceMinZ - swingMinZ
          */
         FrameVector searchDirection = new FrameVector(supportAnkleZUpFrame, 0.0, 0.0, -1.0);
         FramePoint stanceMinZPoint = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(supportFoot.getContactPoints(),
                                         searchDirection, 1).get(0);
         stanceMinZPoint.changeFrame(supportAnkleZUpFrame);
         double stanceMinZ = stanceMinZPoint.getZ();
         double swingMinZ = DesiredFootstepCalculatorTools.computeMinZWithRespectToAnkleInWorldFrame(swingFootToWorldRotation, swingFoot);
         footstepPosition.setZ(footstepPosition.getZ() + stanceMinZ - swingMinZ);
         footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());
      }
      else
      {
         /*
          * use ground profile to determine height at the planned foot position
          */
         
         footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());
         double groundZ = groundProfile.heightAt(footstepPosition.getX(), footstepPosition.getY(), 0.0);
         double ankleHeight = FootstepUtils.getSoleToAnkleHeight(swingFoot);
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

   protected List<FramePoint> getContactPoints(RobotSide swingSide)
   {
      double stepPitch = this.stepPitch.getDoubleValue();
      List<FramePoint> allContactPoints = contactableBodies.get(swingSide).getContactPoints();
      if (stepPitch == 0.0)
      {
         return allContactPoints;
      }
      else
      {
         FrameVector forwardInFootFrame = new FrameVector(contactableBodies.get(swingSide).getBodyFrame());
         ReferenceFrame frame = allContactPoints.get(0).getReferenceFrame();
         forwardInFootFrame.changeFrame(frame);
         forwardInFootFrame.scale(Math.signum(stepPitch));
         int nPoints = 2;

         return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(allContactPoints, forwardInFootFrame, nPoints);
      }
   }
}
