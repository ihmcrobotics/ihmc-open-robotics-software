package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class FootstepPoseHeuristicChecker
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DefaultFootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapperReadOnly snapper;

   private final TransformReferenceFrame startOfSwingFrame = new TransformReferenceFrame("startOfSwingFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame stanceFootFrame = new TransformReferenceFrame("stanceFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame candidateFootFrame = new TransformReferenceFrame("candidateFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame startOfSwingZUpFrame = new ZUpFrame(startOfSwingFrame, "startOfSwingZUpFrame");
   private final ZUpFrame stanceFootZUpFrame = new ZUpFrame(stanceFootFrame, "stanceFootZUpFrame");
   private final FramePose3D stanceFootPose = new FramePose3D();
   private final FramePose3D candidateFootPose = new FramePose3D();

   private final YoFramePoseUsingYawPitchRoll yoStanceFootPose = new YoFramePoseUsingYawPitchRoll("stance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoseUsingYawPitchRoll yoCandidateFootPose = new YoFramePoseUsingYawPitchRoll("candidate", stanceFootZUpFrame, registry);

   private final YoDouble stepWidth = new YoDouble("stepWidth", registry);
   private final YoDouble stepLength = new YoDouble("stepLength", registry);
   private final YoDouble stepHeight = new YoDouble("stepHeight", registry);
   private final YoDouble stepReachXY = new YoDouble("stepReachXY", registry);
   private final YoDouble stepYaw = new YoDouble("stepYaw", registry);
   private final YoDouble swingHeight = new YoDouble("swingHeight", registry);
   private final YoDouble swingReach = new YoDouble("swingReach", registry);

   private final YoBoolean stepIsPitchedBack = new YoBoolean("stepIsPitchedBack", registry);

   public FootstepPoseHeuristicChecker(DefaultFootstepPlannerParametersReadOnly parameters, YoRegistry parentRegistry)
   {
      this(parameters, null, parentRegistry);
   }

   public FootstepPoseHeuristicChecker(DefaultFootstepPlannerParametersReadOnly parameters, FootstepSnapperReadOnly snapper, YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      parentRegistry.addChild(registry);
   }

   public BipedalFootstepPlannerNodeRejectionReason snapAndCheckValidity(DiscreteFootstep candidateStep,
                                                                         DiscreteFootstep stanceStep,
                                                                         DiscreteFootstep startOfSwingStep)
   {
      RobotSide stepSide = candidateStep.getRobotSide();

      FootstepSnapDataReadOnly candidateStepSnapData = snapper.snapFootstep(candidateStep);
      FootstepSnapDataReadOnly stanceStepSnapData = snapper.snapFootstep(stanceStep);

      RigidBodyTransformReadOnly candidateStepTransform = candidateStepSnapData.getSnappedStepTransform(candidateStep);
      RigidBodyTransformReadOnly stanceStepTransform = stanceStepSnapData.getSnappedStepTransform(stanceStep);
      RigidBodyTransformReadOnly startOfSwingTransform = null;

      if (startOfSwingStep != null)
      {
         FootstepSnapDataReadOnly startOfSwingSnapData = snapper.snapFootstep(startOfSwingStep);
         startOfSwingTransform = startOfSwingSnapData.getSnappedStepTransform(startOfSwingStep);
      }

      return checkValidity(stepSide, candidateStepTransform, stanceStepTransform, startOfSwingTransform);
   }

   public BipedalFootstepPlannerNodeRejectionReason checkValidity(RobotSide stepSide,
                                                                  RigidBodyTransformReadOnly candidateStepTransform,
                                                                  RigidBodyTransformReadOnly stanceStepTransform,
                                                                  RigidBodyTransformReadOnly startOfSwingTransform)
   {
      candidateFootFrame.setTransformAndUpdate(candidateStepTransform);
      stanceFootFrame.setTransformAndUpdate(stanceStepTransform);
      stanceFootZUpFrame.update();

      candidateFootPose.setToZero(candidateFootFrame);
      candidateFootPose.changeFrame(stanceFootZUpFrame);
      yoCandidateFootPose.set(candidateFootPose);

      stanceFootPose.setToZero(stanceFootFrame);
      stanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      yoStanceFootPose.set(stanceFootPose);

      stepLength.set(candidateFootPose.getX());
      stepWidth.set(stepSide.negateIfRightSide(candidateFootPose.getY()));
      stepReachXY.set(EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(candidateFootPose.getX()), Math.abs(stepWidth.getValue() - parameters.getIdealFootstepWidth())));
      stepHeight.set(candidateFootPose.getZ());
      double maximumStepZ = parameters.getMaxStepZ();

      Vector3D zAxis = new Vector3D(Axis3D.Z);
      candidateStepTransform.transform(zAxis);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMinSurfaceIncline());
      if (zAxis.getZ() < minimumSurfaceNormalZ)
      {
         return BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP;
      }
      else if (stepWidth.getValue() < parameters.getMinStepWidth())
      {
         return BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_WIDE_ENOUGH;
      }
      else if (stepWidth.getValue() > parameters.getMaxStepWidth())
      {
         return BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_WIDE;
      }
//      else if (stepLength.getValue() < parameters.getMinimumStepLength())
//      {
//         return BipedalFootstepPlannerNodeRejectionReason.STEP_NOT_LONG_ENOUGH;
//      }
      else if (Math.abs(stepHeight.getValue()) > maximumStepZ)
      {
         return BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW;
      }

      double alphaPitchedBack = Math.max(0.0, - stanceFootPose.getPitch() / parameters.getMinSurfaceIncline());

      stepIsPitchedBack.set(alphaPitchedBack > 0.0);

      if (stepReachXY.getValue() > parameters.getMaxStepReach())
      {
         return BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_FAR;
      }

      double maxYaw = parameters.getMaxStepYaw();
      double minYaw = parameters.getMinStepYaw();
      double yawDelta = AngleTools.computeAngleDifferenceMinusPiToPi(candidateStepTransform.getRotation().getYaw(), stanceFootPose.getRotation().getYaw());
      if (!MathTools.intervalContains(stepSide.negateIfRightSide(yawDelta), minYaw, maxYaw))
      {
         return BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH;
      }

      if (startOfSwingTransform == null)
      {
         return null;
      }

      startOfSwingFrame.setTransformAndUpdate(startOfSwingTransform);
      startOfSwingZUpFrame.update();
      candidateFootPose.changeFrame(startOfSwingZUpFrame);
      swingHeight.set(candidateFootPose.getZ());
      swingReach.set(EuclidCoreTools.norm(candidateFootPose.getX(), candidateFootPose.getY()));

      if (Math.abs(swingHeight.getValue()) > parameters.getMaxSwingZ())
      {
         return BipedalFootstepPlannerNodeRejectionReason.SWING_HEIGHT_TOO_LARGE;
      }

      if (swingReach.getValue() > parameters.getMaxSwingReach())
      {
         return BipedalFootstepPlannerNodeRejectionReason.SWING_REACH_TOO_LARGE;
      }

      return null;
   }

   void setApproximateStepDimensions(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep)
   {
      if (candidateStep == null)
      {
         return;
      }

      double dx = candidateStep.getX() - stanceStep.getX();
      double dy = candidateStep.getY() - stanceStep.getY();

      double stepLength = dx * Math.cos(stanceStep.getYaw()) + dy * Math.sin(stanceStep.getYaw());
      double stepWidth = - dx * Math.sin(stanceStep.getYaw()) + dy * Math.cos(stanceStep.getYaw());
      stepWidth = stanceStep.getRobotSide().negateIfLeftSide(stepWidth);

      double stepYaw = AngleTools.computeAngleDifferenceMinusPiToPi(candidateStep.getYaw(), stanceStep.getYaw());
      stepYaw = stanceStep.getRobotSide().negateIfLeftSide(stepYaw);

      this.stepLength.set(stepLength);
      this.stepWidth.set(stepWidth);
      this.stepYaw.set(stepYaw);
   }

   void clearLoggedVariables()
   {
      stepWidth.setToNaN();
      stepLength.setToNaN();
      stepHeight.setToNaN();
      stepReachXY.setToNaN();
      yoStanceFootPose.setToNaN();
      yoCandidateFootPose.setToNaN();

      stepIsPitchedBack.set(false);
   }

   public static void main(String[] args)
   {
      double originalYaw = LatticePoint.gridSizeYaw * 2.0;
      System.out.println(originalYaw);
      DiscreteFootstep footstep = new DiscreteFootstep(0.0, 0.0, originalYaw, RobotSide.LEFT);

      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.1, 0.1);
      footPolygon.addVertex(-0.1, 0.1);
      footPolygon.addVertex(0.1, -0.1);
      footPolygon.addVertex(-0.1, -0.1);
      footPolygon.update();

      ConvexPolygon2D regionPolygon = new ConvexPolygon2D();
      regionPolygon.addVertex(1.0, 1.0);
      regionPolygon.addVertex(-1.0, 1.0);
      regionPolygon.addVertex(1.0, -1.0);
      regionPolygon.addVertex(-1.0, -1.0);
      regionPolygon.update();

      RigidBodyTransformGenerator generator = new RigidBodyTransformGenerator();
      generator.rotate(0.4, Axis3D.X);
      generator.rotate(0.4, Axis3D.Y);
      RigidBodyTransform regionTransform = generator.getRigidBodyTransformCopy();

      PlanarRegion region = new PlanarRegion(regionTransform, List.of(regionPolygon));

      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(footPolygon,
                                                                                                    new PlanarRegionsList(region),
                                                                                                    Double.MAX_VALUE);

      RigidBodyTransform snappedStepTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getSnappedStepTransform(footstep, snapTransform, snappedStepTransform);

      System.out.println(snappedStepTransform.getRotation().getYaw());
   }
}
