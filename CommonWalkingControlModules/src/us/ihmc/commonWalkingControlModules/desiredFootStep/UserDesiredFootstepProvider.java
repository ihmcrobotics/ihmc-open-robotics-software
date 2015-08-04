package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;

public class UserDesiredFootstepProvider implements FootstepProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   private final SideDependentList<ReferenceFrame> ankleZUpReferenceFrames;

   private final String namePrefix = "userStep";

   private final IntegerYoVariable userStepsToTake = new IntegerYoVariable(namePrefix + "sToTake", registry);
   private final EnumYoVariable<RobotSide> userStepFirstSide = new EnumYoVariable<RobotSide>(namePrefix + "FirstSide", registry, RobotSide.class);
   private final DoubleYoVariable userStepLength = new DoubleYoVariable(namePrefix + "Length", registry);
   private final DoubleYoVariable userStepWidth = new DoubleYoVariable(namePrefix + "Width", registry);
   private final DoubleYoVariable userStepSideways = new DoubleYoVariable(namePrefix + "Sideways", registry);
   private final DoubleYoVariable userStepMinWidth = new DoubleYoVariable(namePrefix + "MinWidth", registry);
   private final DoubleYoVariable userStepHeight = new DoubleYoVariable(namePrefix + "Height", registry);
   private final DoubleYoVariable userStepYaw = new DoubleYoVariable(namePrefix + "Yaw", registry);
   private final BooleanYoVariable userStepsTakeEm = new BooleanYoVariable(namePrefix + "sTakeEm", registry);
   private final BooleanYoVariable userStepSquareUp = new BooleanYoVariable(namePrefix + "SquareUp", registry);
   private final IntegerYoVariable userStepsNotifyCompleteCount = new IntegerYoVariable(namePrefix + "sNotifyCompleteCount", registry);

   private final BooleanYoVariable controllerHasCancelledPlan = new BooleanYoVariable(namePrefix + "ControllerHasCancelledPlan", registry);

   private final DoubleYoVariable userStepHeelPercentage = new DoubleYoVariable(namePrefix + "HeelPercentage", registry);
   private final DoubleYoVariable userStepToePercentage = new DoubleYoVariable(namePrefix + "ToePercentage", registry);

   private final ArrayList<Footstep> footstepList = new ArrayList<Footstep>();

   public UserDesiredFootstepProvider(SideDependentList<ContactablePlaneBody> bipedFeet, SideDependentList<ReferenceFrame> ankleZUpReferenceFrames,
         final WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.bipedFeet = bipedFeet;
      this.ankleZUpReferenceFrames = ankleZUpReferenceFrames;

      userStepWidth.set((walkingControllerParameters.getMaxStepWidth() + walkingControllerParameters.getMinStepWidth()) / 2);
      userStepMinWidth.set(walkingControllerParameters.getMinStepWidth());
      userStepFirstSide.set(RobotSide.LEFT);

      userStepHeelPercentage.set(1.0);
      userStepToePercentage.set(1.0);

      userStepLength.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (v.getValueAsDouble() > walkingControllerParameters.getMaxStepLength())
            {
               v.setValueFromDouble(walkingControllerParameters.getMaxStepLength());
            }
         }
      });
   }

   @Override
   public Footstep poll()
   {
      if (userStepsTakeEm.getBooleanValue())
      {
         footstepList.clear();
         userStepsTakeEm.set(false);
         controllerHasCancelledPlan.set(false);

         RobotSide stepSide = userStepFirstSide.getEnumValue();
         if (stepSide == null)
            stepSide = RobotSide.LEFT;

         Footstep previousFootstep = null;

         for (int i = 0; i < userStepsToTake.getIntegerValue(); i++)
         {
            Footstep footstep;

            if (i == 0)
            {
               footstep = createFirstFootstep(stepSide);
            }
            else if (i == userStepsToTake.getIntegerValue() - 1 && userStepSquareUp.getBooleanValue())
            {
               footstep = squareUp(previousFootstep, stepSide);
            }
            else
            {
               footstep = createNextFootstep(previousFootstep, stepSide);
            }

            footstepList.add(footstep);
            previousFootstep = footstep;
            stepSide = stepSide.getOppositeSide();
         }
      }

      if (footstepList.isEmpty())
         return null;

      Footstep ret = footstepList.get(0);
      footstepList.remove(0);

      return ret;
   }

   private Footstep createFirstFootstep(RobotSide swingLegSide)
   {
      RobotSide supportLegSide = swingLegSide.getOppositeSide();

      // Footstep Frame
      ReferenceFrame supportAnkleZUpFrame = ankleZUpReferenceFrames.get(supportLegSide);

      Footstep footstep = createFootstep(supportAnkleZUpFrame, swingLegSide);

      return footstep;
   }

   private Footstep createNextFootstep(Footstep previousFootstep, RobotSide swingLegSide)
   {
      FramePose pose = new FramePose();
      previousFootstep.getPose(pose);
      PoseReferenceFrame referenceFrame = new PoseReferenceFrame("step" + userStepsNotifyCompleteCount.getIntegerValue(), pose);

      return createFootstep(referenceFrame, swingLegSide);
   }

   private Footstep squareUp(Footstep previousFootstep, RobotSide swingLegSide)
   {
      FramePose pose = new FramePose();
      previousFootstep.getPose(pose);
      pose.setY(pose.getY() + swingLegSide.negateIfRightSide(userStepWidth.getDoubleValue()));
      PoseReferenceFrame referenceFrame = new PoseReferenceFrame("step" + userStepsNotifyCompleteCount.getIntegerValue(), pose);

      ContactablePlaneBody foot = bipedFeet.get(swingLegSide);

      boolean trustHeight = false;
      Footstep desiredFootstep = new Footstep(foot.getRigidBody(), swingLegSide, foot.getSoleFrame(), referenceFrame, trustHeight);

      List<FramePoint2d> contactFramePoints = foot.getContactPoints2d();
      ArrayList<Point2d> contactPoints = new ArrayList<Point2d>();

      for (FramePoint2d contactFramePoint : contactFramePoints)
      {
         Point2d contactPoint = contactFramePoint.getPointCopy();

         if (contactFramePoint.getX() > 0.0)
         {
            contactPoint.setX(contactPoint.getX() * userStepToePercentage.getDoubleValue());
         }
         else
         {
            contactPoint.setX(contactPoint.getX() * userStepHeelPercentage.getDoubleValue());
         }

         contactPoints.add(contactPoint);
      }

      desiredFootstep.setPredictedContactPointsFromPoint2ds(contactPoints);
      return desiredFootstep;
   }

   private Footstep createFootstep(ReferenceFrame previousFootFrame, RobotSide swingLegSide)
   {
      RobotSide supportLegSide = swingLegSide.getOppositeSide();

      // Footstep Position
      FramePoint footstepPosition = new FramePoint(previousFootFrame);
      double stepYOffset = supportLegSide.negateIfLeftSide(userStepWidth.getDoubleValue()) + userStepSideways.getDoubleValue();
      if ((supportLegSide == RobotSide.LEFT) && (stepYOffset > -userStepMinWidth.getDoubleValue()))
      {
         stepYOffset = -userStepMinWidth.getDoubleValue();
      }
      if ((supportLegSide == RobotSide.RIGHT) && (stepYOffset < userStepMinWidth.getDoubleValue()))
      {
         stepYOffset = userStepMinWidth.getDoubleValue();
      }

      FrameVector footstepOffset = new FrameVector(previousFootFrame, userStepLength.getDoubleValue(), stepYOffset, userStepHeight.getDoubleValue());

      footstepPosition.add(footstepOffset);

      // Footstep Orientation
      FrameOrientation footstepOrientation = new FrameOrientation(previousFootFrame);
      footstepOrientation.setYawPitchRoll(userStepYaw.getDoubleValue(), 0.0, 0.0);

      // Create a foot Step Pose from Position and Orientation
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);

      ContactablePlaneBody foot = bipedFeet.get(swingLegSide);

      boolean trustHeight = false;
      Footstep desiredFootstep = new Footstep(foot.getRigidBody(), swingLegSide, foot.getSoleFrame(), footstepPoseFrame, trustHeight);

      List<FramePoint2d> contactFramePoints = foot.getContactPoints2d();
      ArrayList<Point2d> contactPoints = new ArrayList<Point2d>();

      for (FramePoint2d contactFramePoint : contactFramePoints)
      {
         Point2d contactPoint = contactFramePoint.getPointCopy();

         if (contactFramePoint.getX() > 0.0)
         {
            contactPoint.setX(contactPoint.getX() * userStepToePercentage.getDoubleValue());
         }
         else
         {
            contactPoint.setX(contactPoint.getX() * userStepHeelPercentage.getDoubleValue());
         }

         contactPoints.add(contactPoint);
      }

      desiredFootstep.setPredictedContactPointsFromPoint2ds(contactPoints);

      return desiredFootstep;
   }

   @Override
   public Footstep peek()
   {
      if (footstepList.isEmpty())
         return null;

      Footstep ret = footstepList.get(0);

      return ret;
   }

   @Override
   public Footstep peekPeek()
   {
      if (footstepList.size() < 2)
         return null;

      Footstep ret = footstepList.get(1);

      return ret;
   }

   @Override
   public boolean isEmpty()
   {
      return footstepList.isEmpty();
   }

   @Override
   public void notifyComplete(FramePose actualFootPoseInWorld)
   {
      userStepsNotifyCompleteCount.increment();
   }

   @Override
   public void notifyWalkingComplete()
   {
   }

   @Override
   public int getNumberOfFootstepsToProvide()
   {
      return footstepList.size();
   }

   @Override
   public boolean isBlindWalking()
   {
      return true;
   }

   @Override
   public boolean isPaused()
   {
      return false;
   }

   @Override
   public void cancelPlan()
   {
      controllerHasCancelledPlan.set(true);
      footstepList.clear();
   }
}