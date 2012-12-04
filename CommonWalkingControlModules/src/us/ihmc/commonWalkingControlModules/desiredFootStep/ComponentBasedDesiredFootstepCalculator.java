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
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComponentBasedDesiredFootstepCalculator extends AbstractAdjustableDesiredFootstepCalculator
{
   private final DoubleYoVariable inPlaceWidth = new DoubleYoVariable("inPlaceWidth", registry);
   private final DoubleYoVariable walkingForwardWidth = new DoubleYoVariable("walkingForwardWidth", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable minStepWidth = new DoubleYoVariable("minStepWidth", registry);
   private final DoubleYoVariable maxStepWidth = new DoubleYoVariable("maxStepWidth", registry);

   private final DoubleYoVariable sidestepMaxWidth = new DoubleYoVariable("sidestepMaxWidth", registry);
   private final DoubleYoVariable sidestepMinWidth = new DoubleYoVariable("sidestepMinWidth", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);

   private final DoubleYoVariable velocityMagnitudeInHeading = new DoubleYoVariable("velocityMagnitudeInHeading", registry);
   private final DoubleYoVariable velocityMagnitudeToLeftOfHeading = new DoubleYoVariable("velocityMagnitudeToLeftOfHeading", registry);

   private final SideDependentList<? extends ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final DesiredVelocityControlModule desiredVelocityControlModule;

   public ComponentBasedDesiredFootstepCalculator(SideDependentList<? extends ReferenceFrame> ankleZUpFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
           DesiredHeadingControlModule desiredHeadingControlModule, DesiredVelocityControlModule desiredVelocityControlModule,
           YoVariableRegistry parentRegistry)
   {
      super(bipedFeet, getFramesToStoreFootstepsIn(), parentRegistry);

      this.ankleZUpFrames = ankleZUpFrames;
      this.bipedFeet = bipedFeet;

      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
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

      velocityMagnitudeInHeading.set(desiredVelocity.dot(desiredHeading));
      velocityMagnitudeToLeftOfHeading.set(desiredVelocity.dot(toLeftOfDesiredHeading));

      double stepForward = maxStepLength.getDoubleValue() * velocityMagnitudeInHeading.getDoubleValue();
      double stepSideways = swingLegSide.negateIfRightSide(inPlaceWidth.getDoubleValue());    // maxStepLength.getDoubleValue() * velocityMagnitudeToLeftOfHeading;

      FrameVector2d desiredVelocityInSupportAnkleZUpFrame = desiredVelocity.changeFrameCopy(supportAnkleZUpFrame);
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

   private FramePoint computeDesiredFootPosition(RobotSide swingLegSide, ReferenceFrame supportAnkleZUpFrame, FrameVector2d desiredOffsetFromAnkle,
           Matrix3d footToWorldRotation)
   {
      desiredOffsetFromAnkle.changeFrame(supportAnkleZUpFrame);
      FramePoint footstepPosition = new FramePoint(supportAnkleZUpFrame, desiredOffsetFromAnkle.getX(), desiredOffsetFromAnkle.getY(), 0.0);
      footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double minZ = DesiredFootstepCalculatorTools.computeMinZWithRespectToAnkleInWorldFrame(footToWorldRotation, bipedFeet.get(swingLegSide));
      footstepPosition.setZ(-minZ);

      return footstepPosition;
   }

   private void setYoVariables(RobotSide swingLegSide, Matrix3d rotation, Vector3d translation)
   {
      footstepOrientations.get(swingLegSide).set(rotation);
      footstepPositions.get(swingLegSide).set(translation);
   }

   public void setupParametersForM2V2()
   {
      setInPlaceWidth(0.25);
      setWalkingForwardWidth(0.17);

      setMaxStepLength(0.4);

      setSidestepMaxWidth(0.3);
      setSidestepMinWidth(0.15);

      setMinStepWidth(0.2);
      setMaxStepWidth(0.4);
   }

   public void setupParametersForR2()
   {
      setInPlaceWidth(0.4);
      setWalkingForwardWidth(0.25);

      setMaxStepLength(0.6);

      setSidestepMaxWidth(0.4);
      setSidestepMinWidth(0.15);

      setMinStepWidth(0.25);
      setMaxStepWidth(0.5);
   }

   public void setInPlaceWidth(double inPlaceWidth)
   {
      this.inPlaceWidth.set(inPlaceWidth);
   }

   public void setWalkingForwardWidth(double walkingForwardWidth)
   {
      this.walkingForwardWidth.set(walkingForwardWidth);
   }

   public void setMaxStepLength(double maxStepLength)
   {
      this.maxStepLength.set(maxStepLength);
   }

   public void setSidestepMaxWidth(double sidestepMaxWidth)
   {
      this.sidestepMaxWidth.set(sidestepMaxWidth);
   }

   public void setSidestepMinWidth(double sidestepMinWidth)
   {
      this.sidestepMinWidth.set(sidestepMinWidth);
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
      List<FramePoint> allContactPoints = bipedFeet.get(swingSide).getContactPoints();
      if (stepPitch == 0.0)
      {
         return allContactPoints;
      }
      else
      {
         FrameVector forwardInFootFrame = new FrameVector(bipedFeet.get(swingSide).getBodyFrame());
         ReferenceFrame frame = allContactPoints.get(0).getReferenceFrame();
         forwardInFootFrame.changeFrame(frame);
         forwardInFootFrame.scale(Math.signum(stepPitch));
         int nPoints = 2;

         return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(allContactPoints, forwardInFootFrame, nPoints);
      }
   }
}
