package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.*;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.vecmath.Point2d;
import java.util.ArrayList;
import java.util.List;

public class UserDesiredPlanarFootstepProvider implements FootstepProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SideDependentList<ContactablePlaneBody> bipedFeet;
   private final SideDependentList<ReferenceFrame> ankleZUpReferenceFrames;

   private final String namePrefix = "userStep";

   private final IntegerYoVariable userStepsToTake = new IntegerYoVariable(namePrefix + "sToTake", registry);
   private final EnumYoVariable<RobotSide> userStepFirstSide = new EnumYoVariable<RobotSide>(namePrefix + "FirstSide", registry, RobotSide.class);
   private final DoubleYoVariable userStepLength = new DoubleYoVariable(namePrefix + "Length", registry);
   private final DoubleYoVariable userStepHeight = new DoubleYoVariable(namePrefix + "Height", registry);
   private final BooleanYoVariable userStepsTakeEm = new BooleanYoVariable(namePrefix + "sTakeEm", registry);
   private final BooleanYoVariable userStepSquareUp = new BooleanYoVariable(namePrefix + "SquareUp", registry);
   private final IntegerYoVariable userStepsNotifyCompleteCount = new IntegerYoVariable(namePrefix + "sNotifyCompleteCount", registry);

   private final BooleanYoVariable controllerHasCancelledPlan = new BooleanYoVariable(namePrefix + "ControllerHasCancelledPlan", registry);

   private final DoubleYoVariable userStepHeelPercentage = new DoubleYoVariable(namePrefix + "HeelPercentage", registry);
   private final DoubleYoVariable userStepToePercentage = new DoubleYoVariable(namePrefix + "ToePercentage", registry);

   private final double userFixedWidth;

   private RobotSide swingSide = RobotSide.LEFT;
   private RobotSide supportSide = swingSide.getOppositeSide();
   private Footstep previousFootstep;
   private Footstep desiredFootstep;
   private ContactablePlaneBody swingFoot;
   private ReferenceFrame footstepReferenceFrame;

   private Footstep nextFootstep;
   private Footstep nextNextFootstep;

   private final FramePoint footstepPosition;
   private final FrameOrientation footstepOrientation;
   private final FrameVector footstepOffset;
   private final FramePose footstepPose;

   private final ArrayList<Footstep> footstepList = new ArrayList<Footstep>();

   public UserDesiredPlanarFootstepProvider(SideDependentList<ContactablePlaneBody> bipedFeet, SideDependentList<ReferenceFrame> ankleZUpReferenceFrames,
         final WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.bipedFeet = bipedFeet;
      this.ankleZUpReferenceFrames = ankleZUpReferenceFrames;

      swingFoot = bipedFeet.get(swingSide);
      ReferenceFrame stanceFootFrame = bipedFeet.get(swingSide).getSoleFrame();
      footstepPosition = new FramePoint(stanceFootFrame);
      footstepOrientation = new FrameOrientation(stanceFootFrame);
      footstepOffset = new FrameVector(stanceFootFrame);
      footstepPose = new FramePose(stanceFootFrame);

      userStepFirstSide.set(RobotSide.LEFT);

      userStepHeelPercentage.set(1.0);
      userStepToePercentage.set(1.0);

      userFixedWidth = walkingControllerParameters.getMinStepWidth();

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

         swingSide = userStepFirstSide.getEnumValue();
         if (swingSide == null)
            swingSide = RobotSide.LEFT;

         supportSide = swingSide.getOppositeSide();
         previousFootstep = null;

         for (int i = 0; i < userStepsToTake.getIntegerValue(); i++)
         {
            swingFoot = bipedFeet.get(swingSide);
            if (i == userStepsToTake.getIntegerValue() - 1 && userStepSquareUp.getBooleanValue())
            {
               squareUp();
            }
            else
            {
               createNextFootstep();
            }

            footstepList.add(desiredFootstep);
            previousFootstep = desiredFootstep;
            swingSide = swingSide.getOppositeSide();
            supportSide = swingSide.getOppositeSide();
         }
      }

      if (footstepList.isEmpty())
         return null;

      if (footstepList.size() > 0)
      {
         nextFootstep = footstepList.get(0);
         if (footstepList.size() > 1)
            nextNextFootstep = footstepList.get(1);
         else
            nextNextFootstep = null;
      }
      else
      {
         nextFootstep = null;
         nextNextFootstep = null;
      }
      footstepList.remove(0);

      return nextFootstep;
   }

   private void createNextFootstep()
   {
      if (previousFootstep != null)
      {
         footstepReferenceFrame = previousFootstep.getPoseReferenceFrame();
      }
      else
      {
         footstepReferenceFrame = ankleZUpReferenceFrames.get(supportSide);
      }

      createFootstep();
   }

   private void squareUp()
   {
      if (previousFootstep != null)
      {
         footstepReferenceFrame = previousFootstep.getPoseReferenceFrame();
      }
      else
      {
         footstepReferenceFrame = ankleZUpReferenceFrames.get(supportSide);
      }

      createFootstep(0.0, 0.0);
   }

   private void createFootstep()
   {
      createFootstep(userStepLength.getDoubleValue(), userStepHeight.getDoubleValue());
   }

   private void createFootstep(double userStepLength, double userStepHeight)
   {
      // Footstep Position
      footstepPosition.setToZero(footstepReferenceFrame);
      footstepOffset.setToZero(footstepReferenceFrame);
      footstepOffset.set(userStepLength, supportSide.negateIfLeftSide(userFixedWidth), userStepHeight);
      footstepPosition.add(footstepOffset);

      // Footstep Orientation
      footstepOrientation.setToZero(footstepReferenceFrame);

      // Create a foot Step Pose from Position and Orientation
      footstepPose.setToZero(footstepReferenceFrame);
      footstepPose.setPosition(footstepPosition);
      footstepPose.setOrientation(footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);

      boolean trustHeight = false;
      desiredFootstep = new Footstep(swingFoot.getRigidBody(), swingSide, swingFoot.getSoleFrame(), footstepPoseFrame, trustHeight);

      List<FramePoint2d> contactFramePoints = swingFoot.getContactPoints2d();
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
   }

   @Override
   public Footstep peek()
   {
      return nextFootstep;
   }

   @Override
   public Footstep peekPeek()
   {
      return nextNextFootstep;
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