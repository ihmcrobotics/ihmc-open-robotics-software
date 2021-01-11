package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoPTrajectoryGeneratorState extends YoSaveableModuleState
{
   private final YoPreallocatedList<DynamicPlanningFootstep> footsteps;
   private final YoPreallocatedList<PlanningTiming> footstepTimings;

   private final YoDouble finalTransferDuration;
   private final YoDouble percentageStandingWeightDistributionOnLeftFoot;

   private final YoFramePoint2D initialCoP;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSole = new SideDependentList<>();
   private final SideDependentList<FixedFramePose3DBasics> footPoses = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> soleContactFrames = new SideDependentList<>();

   public CoPTrajectoryGeneratorState(YoRegistry registry)
   {
      footsteps = new YoPreallocatedList<>(DynamicPlanningFootstep.class, () -> createFootstep(registry), "footstep", registry, CoPTrajectoryParameters.maxNumberOfStepsToConsider);
      footstepTimings = new YoPreallocatedList<>(PlanningTiming.class, () -> createTiming(registry), "footstepTiming", registry, CoPTrajectoryParameters.maxNumberOfStepsToConsider);
      registerVariableToSave(footsteps.getYoPosition());
      registerVariableToSave(footstepTimings.getYoPosition());

      finalTransferDuration = new YoDouble("finalTransferDuration", registry);
      percentageStandingWeightDistributionOnLeftFoot = new YoDouble("percentageStandingWeightDistributionOnLeftFoot", registry);
      registerVariableToSave(finalTransferDuration);
      registerVariableToSave(percentageStandingWeightDistributionOnLeftFoot);

      percentageStandingWeightDistributionOnLeftFoot.set(0.5);

      initialCoP = new YoFramePoint2D("initialCoP", ReferenceFrame.getWorldFrame(), registry);
      YoSaveableModuleStateTools.registerYoTuple2DToSave(initialCoP, this);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePose3D footPose = new YoFramePose3D(robotSide.getCamelCaseName() + "FootPose", ReferenceFrame.getWorldFrame(), registry);
         YoSaveableModuleStateTools.registerYoFramePose3DToSave(footPose, this);
         footPoses.put(robotSide, footPose);

         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getCamelCaseName() + "SoleFrame", footPose);
         soleContactFrames.put(robotSide, soleFrame);

         footPose.attachVariableChangedListener(v -> soleFrame.setPoseAndUpdate(footPose));

         List<YoFramePoint2D> vertexBuffer = new ArrayList<>();
         String prefix = robotSide.getCamelCaseName() + "FootPolygonInSole";
         for (int i = 0; i < 6; i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "_" + i, soleFrame, registry);
            YoSaveableModuleStateTools.registerYoTuple2DToSave(vertex, this);
            vertexBuffer.add(vertex);
         }
         YoInteger numberOfVertices = new YoInteger(prefix + "NumVertices", registry);
         registerVariableToSave(numberOfVertices);
         YoFrameConvexPolygon2D footPolygonInSole = new YoFrameConvexPolygon2D(vertexBuffer,
                                                                               numberOfVertices,
                                                                               soleFrame);
         footPolygonInSole.clearAndUpdate();
         footPolygonsInSole.put(robotSide, footPolygonInSole);
      }
   }

   public void initializeStance(SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames,
                                SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      for (RobotSide robotSide : RobotSide.values)
         initializeStance(robotSide, feetInSoleZUpFrames.get(robotSide), soleFrames.get(robotSide));
   }

   public void initializeStance(RobotSide robotSide, FrameConvexPolygon2DReadOnly supportPolygon, ReferenceFrame soleFrame)
   {
      footPoses.get(robotSide).setFromReferenceFrame(soleFrame);
      footPolygonsInSole.get(robotSide).setMatchingFrame(supportPolygon, false);
   }

   public int getNumberOfFootstep()
   {
      return footsteps.size();
   }

   public FramePoint2DReadOnly getInitialCoP()
   {
      return initialCoP;
   }

   public DynamicPlanningFootstep getFootstep(int index)
   {
      return footsteps.get(index);
   }

   public PlanningTiming getTiming(int index)
   {
      return footstepTimings.get(index);
   }

   public FramePose3DReadOnly getFootPose(RobotSide robotSide)
   {
      return footPoses.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getFootPolygonInSole(RobotSide robotSide)
   {
      return footPolygonsInSole.get(robotSide);
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration.getDoubleValue();
   }

   public double getPercentageStandingWeightDistributionOnLeftFoot()
   {
      return percentageStandingWeightDistributionOnLeftFoot.getValue();
   }

   public void setFinalTransferDuration(double transferDuration)
   {
      finalTransferDuration.set(transferDuration);
   }

   public void setPercentageStandingWeightDistributionOnLeftFoot(double percentageStandingWeightDistributionOnLeftFoot)
   {
      this.percentageStandingWeightDistributionOnLeftFoot.set(percentageStandingWeightDistributionOnLeftFoot);
   }

   public void clear()
   {
      for (int i = 0; i < footsteps.size(); i++)
         footsteps.get(i).clear();
      for (int i = 0; i < footstepTimings.size(); i++)
         footstepTimings.get(i).clear();
      footsteps.clear();
      footstepTimings.clear();
   }

   public void addFootstep(Footstep footstep)
   {
      if (footsteps.size() < footsteps.capacity())
         footsteps.add().set(footstep);
   }

   public void addFootstepTiming(FootstepTiming timing)
   {
      if (footstepTimings.size() < footstepTimings.capacity())
         footstepTimings.add().set(timing);
   }

   public void setInitialCoP(FramePoint3DReadOnly initialCoP)
   {
      this.initialCoP.set(initialCoP);
   }

   public void setInitialCoP(FramePoint2DReadOnly initialCoP)
   {
      this.initialCoP.set(initialCoP);
   }

   private int footstepCounter = 0;

   private DynamicPlanningFootstep createFootstep(YoRegistry registry)
   {
      DynamicPlanningFootstep footstep = new DynamicPlanningFootstep("" + footstepCounter++, registry);
      registerStateToSave(footstep);
      return footstep;
   }

   private int timingCounter = 0;

   private PlanningTiming createTiming(YoRegistry registry)
   {
      PlanningTiming timing = new PlanningTiming("" + timingCounter++, registry);
      registerStateToSave(timing);
      return timing;
   }

   private int shiftFractionCounter = 0;

   private PlanningShiftFraction createShiftFractions(YoRegistry registry)
   {
      PlanningShiftFraction shiftFractions = new PlanningShiftFraction("" + shiftFractionCounter++, registry);
      registerStateToSave(shiftFractions);
      return shiftFractions;
   }
}
