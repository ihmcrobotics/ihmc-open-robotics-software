package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepUtils;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.OverheadPath;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class AbstractFootstepGenerator implements FootstepGenerator
{
   protected static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private static final boolean USE_MASK = true;
   private static final boolean VERBOSE_DEBUG = false;
   private static final boolean VERBOSE_ERROR_PRINTS = false;

   protected static double noTranslationTolerance = 1e-14;

   protected RobotSide stanceStartSidePreference;
   protected RobotSide closestSideToEnd;
   protected RobotSide stanceStartSide;
   protected RobotSide currentFootstepSide;
   protected SideDependentList<RigidBodyBasics> feet;
   protected SideDependentList<ReferenceFrame> soleFrames;
   protected boolean startStancePreferenceSpecified = false;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   protected SideDependentList<? extends ContactablePlaneBody> contactableFeet;
//   protected final FootstepSnapper footstepSnapper = new SimpleFootstepSnapper();

   private SideDependentList<Footstep> priorStanceFeet;
   boolean priorStanceFeetSpecified = false;
   protected double initialDeltaFeetYaw;
   protected double initialDeltaFeetY;
   protected double initialDeltaFeetX;
   protected final FramePose2D startPose = new FramePose2D();

   public AbstractFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
   }

   public AbstractFootstepGenerator(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, RobotSide stanceStart)
   {
      this(feet, soleFrames);
      setStanceStartPreference(stanceStart);
   }

   @Override
   public ArrayList<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> ret = new ArrayList<Footstep>();
      initialize();
      generateFootsteps(ret);

      return ret;
   }

   public void initialize()
   {
      calculateInitialFootPoseAndOffsets();
      initialize(startPose);
   }

   protected abstract void initialize(FramePose2D startPose);

   abstract protected void generateFootsteps(ArrayList<Footstep> ret);

   protected Footstep createFootstep(RobotSide currentFootstepSide, FramePose2D solePose)
   {
      RigidBodyBasics foot = feet.get(currentFootstepSide);
      ReferenceFrame soleFrame = soleFrames.get(currentFootstepSide);

      Footstep footstep;
            FramePoint3D soleFrameInWorldPoint = new FramePoint3D(soleFrame);
            soleFrameInWorldPoint.changeFrame(WORLD_FRAME);
            footstep = generateFootstepWithoutHeightMap(solePose, foot, soleFrame, currentFootstepSide, soleFrameInWorldPoint.getZ(), new Vector3D(0.0, 0.0, 1.0));
            if (VERBOSE_ERROR_PRINTS)
               System.err.println("AbstractFootstepGenerator: Grid data unavailable. Using best guess for ground height.");

      return footstep;
   }

   protected Footstep createFootstepPlacedAtBipedfootLocation(RobotSide side)
   {
      ReferenceFrame soleFrame = soleFrames.get(side);
      Vector3D translation = new Vector3D();
      Quaternion rotation = new Quaternion();
      translation.set(soleFrame.getTransformToWorldFrame().getTranslation());
      rotation.set(soleFrame.getTransformToWorldFrame().getRotation());

      FramePose2D solePose2d = new FramePose2D(soleFrame, new Point2D(translation.getX(), translation.getY()), rotation.getYaw());
      Footstep foot = createFootstep(side, solePose2d);

      return foot;
   }

   protected RobotSide getFarSideFootstep()
   {
      if (priorStanceFeetSpecified)
         closestSideToEnd = determineClosestFootSideToEnd(priorStanceFeet);
      else
         closestSideToEnd = determineClosestBipedFootSideToEnd();

      stanceStartSidePreference = closestSideToEnd;

      return stanceStartSidePreference.getOppositeSide(); // currentFootstepSide could be overridden in footstep generator
   }

   public void setStanceStartPreference(RobotSide stanceStart)
   {
      stanceStartSidePreference = stanceStart;
      if (stanceStart == null)
         startStancePreferenceSpecified = false;
      else
         startStancePreferenceSpecified = true;
   }

   public void setPriorFootposes(SideDependentList<Footstep> priorStanceFeet)
   {
      this.priorStanceFeet = priorStanceFeet;
      if (priorStanceFeet == null)
         priorStanceFeetSpecified = false;
      else
         priorStanceFeetSpecified = true;
   }

   private void calculateInitialFootPoseAndOffsets()
   {
      SideDependentList<Footstep> currentFootsteps;
      if (priorStanceFeetSpecified)
         currentFootsteps = priorStanceFeet;
      else
         currentFootsteps = createFootstepsFromBipedFeet();

      Footstep left = currentFootsteps.get(RobotSide.LEFT);
      Footstep right = currentFootsteps.get(RobotSide.RIGHT);

      FramePose3D leftPose = new FramePose3D();
      FramePose3D rightPose = new FramePose3D();
      left.getPose(leftPose);
      right.getPose(rightPose);

      FramePose2D leftPose2d = new FramePose2D(leftPose);
      FramePose2D rightPose2d = new FramePose2D(rightPose);

      startPose.interpolate(leftPose2d, rightPose2d, 0.5);
      Pose2dReferenceFrame startFramePose = new Pose2dReferenceFrame("StartPoseFrame", startPose);

      leftPose.changeFrame(startFramePose);
      rightPose.changeFrame(startFramePose);
      FrameOrientation2D leftOrientation = new FrameOrientation2D(leftPose.getOrientation());
      FrameOrientation2D rightOrientation = new FrameOrientation2D(rightPose.getOrientation());
      initialDeltaFeetYaw = leftOrientation.difference(rightOrientation);
      initialDeltaFeetY = leftPose.getY() - rightPose.getY();
      initialDeltaFeetX = leftPose.getX() - rightPose.getX();
   }

   protected RobotSide determineClosestBipedFootSideToEnd()
   {
      SideDependentList<Footstep> currentFootsteps = createFootstepsFromBipedFeet();

      return determineClosestFootSideToEnd(currentFootsteps);
   }

   private SideDependentList<Footstep> createFootstepsFromBipedFeet()
   {
      SideDependentList<Footstep> currentFootsteps = new SideDependentList<Footstep>(createFootstepPlacedAtBipedfootLocation(RobotSide.LEFT),
            createFootstepPlacedAtBipedfootLocation(RobotSide.RIGHT));

      return currentFootsteps;
   }

   protected RobotSide determineClosestFootSideToEnd(SideDependentList<Footstep> lastFootsteps)
   {
      FramePose2D endPose = getPath().getPoseAtS(1);
      Point3D endPoint = new Point3D(endPose.getX(), endPose.getY(), 0.0);
      RobotSide closestSideToEnd = FootstepUtils.getFrontFootRobotSideFromFootsteps(lastFootsteps, new FramePoint3D(ReferenceFrame.getWorldFrame(), endPoint));

      return closestSideToEnd;
   }

   protected FramePoint2D offsetFootstepFromPath(RobotSide currentFootstepSide, FramePoint2D footstepPosition2d, double footHeading, double offsetAmount)
   {
      double sideWaysHeading = footHeading + Math.PI / 2.0;
      FrameVector2D offsetVector = new FrameVector2D(WORLD_FRAME, Math.cos(sideWaysHeading), Math.sin(sideWaysHeading));
      offsetVector.scale(currentFootstepSide.negateIfRightSide(offsetAmount));
      FramePoint2D footstepPosition = new FramePoint2D(footstepPosition2d);
      footstepPosition.changeFrame(WORLD_FRAME);
      footstepPosition.add(offsetVector);

      return footstepPosition;
   }

   protected abstract OverheadPath getPath();

   protected RobotSide sideOfHipAngleOpeningStep(double deltaYaw)
   {
      RobotSide footToStepWith = (deltaYaw >= 0) ? RobotSide.LEFT : RobotSide.RIGHT;

      return footToStepWith;
   }

   public static void setNoTranslationTolerance(double noTranslationToleranceValue)
   {
      noTranslationTolerance = noTranslationToleranceValue;
   }

   public static double getNoTranslationTolerance()
   {
      return noTranslationTolerance;
   }

}
