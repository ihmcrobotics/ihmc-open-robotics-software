package us.ihmc.commonWalkingControlModules.desiredFootStep;

import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.terrain.VaryingStairGroundProfile;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;


//TODO: currently only works when the stairs are oriented in the x direction in world frame
public class VaryingStairDesiredFootstepCalculator extends SimpleWorldDesiredFootstepCalculator
{
   private final DoubleYoVariable stepXAfterNosing = new DoubleYoVariable("stepXAfterNosing", registry);
   private final DoubleYoVariable stepLengthOnFlatGround = new DoubleYoVariable("stepLengthOnFlatGround", registry);
   private final DoubleYoVariable stepLengthMinimum = new DoubleYoVariable("stepLengthMinimum", registry);

   private final VaryingStairGroundProfile groundProfile;


   public VaryingStairDesiredFootstepCalculator(VaryingStairGroundProfile groundProfile, SideDependentList<ContactablePlaneBody> bipedFeet,
           CommonHumanoidReferenceFrames referenceFrames, DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry)
   {
      super(bipedFeet, referenceFrames, desiredHeadingControlModule, parentRegistry);
      this.groundProfile = groundProfile;
   }

   @Override
   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      RigidBodyTransform footToWorldTransform = referenceFrames.getFootFrame(supportLegSide).getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());

      FramePoint maxStanceXPoint = DesiredFootstepCalculatorTools.computeMaxXPointInFrame(footToWorldTransform, bipedFeet.get(supportLegSide),
                                      ReferenceFrame.getWorldFrame());
      double maxStanceX = maxStanceXPoint.getX();

      double oneStepLookAheadX = maxStanceX + stepLengthOnFlatGround.getDoubleValue();
      double twoStepLookAheadX = oneStepLookAheadX + stepLengthOnFlatGround.getDoubleValue();

      int stepNumber = groundProfile.computeStepNumber(maxStanceX);
      double stepX = groundProfile.computeStepStartX(stepNumber);

      if (oneStepLookAheadX > stepX)
      {
         oneStepLookAheadX = stepX + stepXAfterNosing.getDoubleValue();
      }
      else
      {
         if (twoStepLookAheadX > stepX)
         {
            twoStepLookAheadX = stepX + stepXAfterNosing.getDoubleValue();
            oneStepLookAheadX = (twoStepLookAheadX + maxStanceX) / 2.0;
         }
      }

      double oneStepLookAheadZ = groundProfile.heightAt(oneStepLookAheadX, 0.0, 0.0);

      stepLength.set(oneStepLookAheadX - maxStanceX);

      double stanceZ = groundProfile.heightAt(maxStanceXPoint.getX(), maxStanceXPoint.getY(), maxStanceXPoint.getZ());
      stepHeight.set(oneStepLookAheadZ - stanceZ);

      FramePoint stanceAnkle = new FramePoint(referenceFrames.getFootFrame(supportLegSide));
      stanceAnkle.changeFrame(ReferenceFrame.getWorldFrame());


      // roll and pitch with respect to world, yaw with respect to other foot's yaw
      RobotSide swingLegSide = supportLegSide.getOppositeSide();
      double swingFootYaw = footstepOrientations.get(supportLegSide).getYaw().getDoubleValue() + stepYaw.getDoubleValue();
      double swingFootPitch = stepPitch.getDoubleValue();
      double swingFootRoll = stepRoll.getDoubleValue();
      footstepOrientations.get(swingLegSide).setYawPitchRoll(swingFootYaw, swingFootPitch, swingFootRoll);
      Matrix3d footToWorldRotation = new Matrix3d();
      footstepOrientations.get(swingLegSide).getMatrix3d(footToWorldRotation);
      double swingMinZWithRespectToAnkle = DesiredFootstepCalculatorTools.computeMinZPointWithRespectToAnkleInWorldFrame(footToWorldRotation,
                                              bipedFeet.get(swingLegSide));
      double swingMaxXWithRespectToAnkle = DesiredFootstepCalculatorTools.computeMaxXWithRespectToAnkleInFrame(footToWorldRotation,
                                              bipedFeet.get(swingLegSide), desiredHeadingControlModule.getDesiredHeadingFrame());


      double swingAnkleX = oneStepLookAheadX - swingMaxXWithRespectToAnkle;    // + addToStepLength.getDoubleValue();
      double swingAnkleY = stanceAnkle.getY() + supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue());
      double swingAnkleZ = oneStepLookAheadZ - swingMinZWithRespectToAnkle;    // + addToStepHeight.getDoubleValue();

      FramePoint newFootstepPosition = new FramePoint(ReferenceFrame.getWorldFrame(), swingAnkleX, swingAnkleY, swingAnkleZ);
      footstepPositions.get(swingLegSide).set(newFootstepPosition);
   }

   @Override
   public void setupParametersForR2InverseDynamics()
   {
      stepXAfterNosing.set(0.05);    // 035);
      stepLengthOnFlatGround.set(0.35);
      stepLengthMinimum.set(0.15);
      stepWidth.set(0.2);
      stepYaw.set(0.0);
      stepPitch.set(0.2);
      stepRoll.set(0.0);
   }
}
