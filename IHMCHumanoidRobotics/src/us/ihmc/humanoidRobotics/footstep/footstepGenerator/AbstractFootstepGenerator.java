package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepUtils;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath.OverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.AtlasFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.ConvexHullFootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepValueFunction;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

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
   protected SideDependentList<RigidBody> feet;
   protected SideDependentList<ReferenceFrame> soleFrames;
   protected boolean startStancePreferenceSpecified = false;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected HeightMapWithPoints heightMap;
   protected SideDependentList<? extends ContactablePlaneBody> contactableFeet;
//   protected final FootstepSnapper footstepSnapper = new SimpleFootstepSnapper();

   //TODO: Fix so not specific to Atlas...
   private final FootstepSnappingParameters snappingParameters = new AtlasFootstepSnappingParameters();
   private final FootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);

   private SideDependentList<Footstep> priorStanceFeet;
   boolean priorStanceFeetSpecified = false;
   protected double initialDeltaFeetYaw;
   protected double initialDeltaFeetY;
   protected double initialDeltaFeetX;
   protected final FramePose2d startPose = new FramePose2d();

   public AbstractFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
   }

   public AbstractFootstepGenerator(SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames, RobotSide stanceStart)
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

   protected abstract void initialize(FramePose2d startPose);

   abstract protected void generateFootsteps(ArrayList<Footstep> ret);

   protected Footstep createFootstep(RobotSide currentFootstepSide, FramePose2d solePose)
   {
      RigidBody foot = feet.get(currentFootstepSide);
      ReferenceFrame soleFrame = soleFrames.get(currentFootstepSide);

      Footstep footstep;
      try
      {
         if (heightMap != null)
         {
            footstep = footstepSnapper.generateFootstepUsingHeightMap(solePose, foot, soleFrame, currentFootstepSide, heightMap);
         }
         else
         {
            FramePoint soleFrameInWorldPoint = new FramePoint(soleFrame);
            soleFrameInWorldPoint.changeFrame(WORLD_FRAME);
            footstep = footstepSnapper.generateFootstepWithoutHeightMap(solePose, foot, soleFrame, currentFootstepSide, soleFrameInWorldPoint.getZ(), new Vector3d(0.0, 0.0, 1.0));
            if (VERBOSE_ERROR_PRINTS)
               System.err.println("AbstractFootstepGenerator: Grid data unavailable. Using best guess for ground height.");
         }
      }
      catch (InsufficientDataException e)
      {
         footstep = footstepSnapper.generateFootstepWithoutHeightMap(solePose, foot, soleFrame, currentFootstepSide, 0, new Vector3d(0.0, 0.0, 1.0));

         if (VERBOSE_DEBUG)
         {
            System.err.println("AbstractFootstepGenerator: No lidar data found for step. Using best guess.");
         }
      }

      return footstep;
   }

   protected Footstep createFootstepPlacedAtBipedfootLocation(RobotSide side)
   {
      ReferenceFrame soleFrame = soleFrames.get(side);
      Vector3d translation = new Vector3d();
      Quat4d rotation = new Quat4d();
      soleFrame.getTransformToWorldFrame().getTranslation(translation);
      soleFrame.getTransformToWorldFrame().getRotation(rotation);

      FramePose2d solePose2d = new FramePose2d(soleFrame, new Point2d(translation.getX(), translation.getY()), RotationTools.computeYaw(rotation));
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

      FramePose leftPose = new FramePose();
      FramePose rightPose = new FramePose();
      left.getSolePose(leftPose);
      right.getSolePose(rightPose);

      FramePose2d leftPose2d = new FramePose2d();
      FramePose2d rightPose2d = new FramePose2d();
      leftPose.getPose2dIncludingFrame(leftPose2d);
      rightPose.getPose2dIncludingFrame(rightPose2d);

      startPose.interpolate(leftPose2d, rightPose2d, 0.5);
      Pose2dReferenceFrame startFramePose = new Pose2dReferenceFrame("StartPoseFrame", startPose);

      leftPose.changeFrame(startFramePose);
      rightPose.changeFrame(startFramePose);
      FrameOrientation2d leftOrientation = new FrameOrientation2d();
      FrameOrientation2d rightOrientation = new FrameOrientation2d();
      leftPose.getOrientation2dIncludingFrame(leftOrientation);
      rightPose.getOrientation2dIncludingFrame(rightOrientation);
      initialDeltaFeetYaw = leftOrientation.sub(rightOrientation);
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
      FramePose2d endPose = getPath().getPoseAtS(1);
      Point3d endPoint = new Point3d(endPose.getX(), endPose.getY(), 0.0);
      RobotSide closestSideToEnd = FootstepUtils.getFrontFootRobotSideFromFootsteps(lastFootsteps, new FramePoint(ReferenceFrame.getWorldFrame(), endPoint));

      return closestSideToEnd;
   }

   protected FramePoint2d offsetFootstepFromPath(RobotSide currentFootstepSide, FramePoint2d footstepPosition2d, double footHeading, double offsetAmount)
   {
      double sideWaysHeading = footHeading + Math.PI / 2.0;
      FrameVector2d offsetVector = new FrameVector2d(WORLD_FRAME, Math.cos(sideWaysHeading), Math.sin(sideWaysHeading));
      offsetVector.scale(currentFootstepSide.negateIfRightSide(offsetAmount));
      FramePoint2d footstepPosition = new FramePoint2d(footstepPosition2d);
      footstepPosition.changeFrame(WORLD_FRAME);
      footstepPosition.add(offsetVector);

      return footstepPosition;
   }

   protected abstract OverheadPath getPath();

   public void setHeightMap(HeightMapWithPoints heightMap, SideDependentList<? extends ContactablePlaneBody> contactableFeet)
   {
      this.heightMap = heightMap;
      this.contactableFeet = contactableFeet;
   }

   public void setPoseFinderParams(double kernelMaskSafetyBuffer, double kernelSize)
   {
      footstepSnapper.setUseMask(USE_MASK, kernelMaskSafetyBuffer, kernelSize);
   }

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
