package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class FootstepProviderTestHelper
{
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final SideDependentList<ReferenceFrame> ankleFrames;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;
   private final double dt;

   public FootstepProviderTestHelper(SideDependentList<? extends ContactablePlaneBody> contactableFeet, SideDependentList<ReferenceFrame> ankleZUpFrames,
                                     SideDependentList<ReferenceFrame> ankleFrames, double dt)
   {
      this.contactableFeet = contactableFeet;
      this.ankleZUpFrames = ankleZUpFrames;
      this.ankleFrames = ankleFrames;
      this.dt = dt;
   }

   public FootstepProvider createFootsteps(double stepWidth, double stepLength, int numberOfSteps)
   {
      final ArrayList<Footstep> footsteps = new ArrayList<>();

      RobotSide currentSide = RobotSide.LEFT;

      FramePoint lastFootstepPosition = new FramePoint(ankleFrames.get(currentSide));
      lastFootstepPosition.changeFrame(worldFrame);

      for (int i = 0; i < numberOfSteps; i++)
      {
         currentSide = currentSide.getOppositeSide();
         double x = lastFootstepPosition.getX();
         if (i < numberOfSteps - 1)
            x += stepLength;
         double y = lastFootstepPosition.getY() + currentSide.negateIfRightSide(stepWidth);
         Footstep footstep = createFootstep(currentSide, x, y);
         footstep.getPositionIncludingFrame(lastFootstepPosition);
         lastFootstepPosition.changeFrame(worldFrame);
         footsteps.add(footstep);
      }

      FootstepProvider footstepProvider = createFootstepProviderForGivenFootstepList(footsteps);

      return footstepProvider;
   }

   public FootstepProvider createFootstepProviderForGivenFootstepList(final ArrayList<Footstep> footsteps)
   {
      FootstepProvider footstepProvider = new FootstepProvider()
      {
         @Override
         public Footstep poll()
         {
            if (footsteps.isEmpty())
               return null;
            else
               return footsteps.remove(0);
         }

         @Override
         public Footstep peek()
         {
            if (footsteps.isEmpty())
               return null;
            else
               return footsteps.get(0);
         }

         @Override
         public Footstep peekPeek()
         {
            if (footsteps.size() <= 1)
               return null;
            else
               return footsteps.get(1);
         }

         @Override
         public void notifyWalkingComplete()
         {
         }

         @Override
         public void notifyComplete(FramePose footPoseInWorld)
         {
         }

         @Override
         public boolean isPaused()
         {
            return false;
         }

         @Override
         public boolean isEmpty()
         {
            return footsteps.isEmpty();
         }

         @Override
         public boolean isBlindWalking()
         {
            return false;
         }

         @Override
         public int getNumberOfFootstepsToProvide()
         {
            return footsteps.size();
         }

         @Override
         public void cancelPlan()
         {
         }
      };

      return footstepProvider;
   }

   private Footstep createFootstep(RobotSide robotSide, double x, double y)
   {
      return createFootstep(robotSide, new Point3d(x, y, 0.0), new Quat4d(0.0, 0.0, 0.0, 1.0));
   }

   public Footstep createFootstep(RobotSide robotSide, Point3d position, Quat4d orientation)
   {
      FramePose footstepPose = new FramePose();
      footstepPose.setPose(position, orientation);

      return createFootstep(robotSide, footstepPose);
   }
   
   public Footstep createFootstep(RobotSide robotSide, Point3d position, double[] orientationYawPitchRoll)
   {
      FramePose footstepPose = new FramePose();      
      footstepPose.setPosition(position);
      footstepPose.setOrientation(orientationYawPitchRoll);
      
      return createFootstep(robotSide, footstepPose);
   }
   
   public Footstep createFootstep(RobotSide robotSide, FramePose footstepPose)
   {
      RigidBody foot = contactableFeet.get(robotSide).getRigidBody();
      ReferenceFrame soleFrame = contactableFeet.get(robotSide).getSoleFrame();
      Footstep ret = new Footstep(foot, robotSide, soleFrame);
      ret.setPose(footstepPose);
      ret.setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(robotSide).getContactPoints2d());

      return ret;
   }

   public FootstepProvider createFlatGroundWalkingTrackFootstepProvider(YoVariableRegistry registry, ArrayList<Updatable> updatables)
   {
      ManualDesiredVelocityControlModule desiredVelocityControlModule = new ManualDesiredVelocityControlModule(worldFrame, registry);
      desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(worldFrame, 1.0, 0.0));

      SimpleDesiredHeadingControlModule desiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, dt, registry);
      desiredHeadingControlModule.setMaxHeadingDot(0.4);
      desiredHeadingControlModule.updateDesiredHeadingFrame();
      boolean cycleThroughAllEvents = true;
      HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(cycleThroughAllEvents, dt,
                                                                                 desiredHeadingControlModule, desiredVelocityControlModule, registry);
      updatables.add(headingAndVelocityEvaluationScript);

      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator = new ComponentBasedDesiredFootstepCalculator(0.084, null, ankleZUpFrames, ankleFrames,
                                                                             contactableFeet, desiredHeadingControlModule, desiredVelocityControlModule,
                                                                             registry);

      double inPlaceWidth = 0.25;
      double maxStepLength = 0.6;
      double minStepWidth = 0.15;
      double maxStepWidth = 0.5;
      double stepPitch = 0.0;

      desiredFootstepCalculator.setInPlaceWidth(inPlaceWidth);
      desiredFootstepCalculator.setMaxStepLength(maxStepLength);
      desiredFootstepCalculator.setMinStepWidth(minStepWidth);
      desiredFootstepCalculator.setMaxStepWidth(maxStepWidth);
      desiredFootstepCalculator.setStepPitch(stepPitch);

      DesiredFootstepCalculatorFootstepProviderWrapper footstepProvider = new DesiredFootstepCalculatorFootstepProviderWrapper(desiredFootstepCalculator,
                                                                             registry);
      footstepProvider.setWalk(true);

      return footstepProvider;
   }
}
