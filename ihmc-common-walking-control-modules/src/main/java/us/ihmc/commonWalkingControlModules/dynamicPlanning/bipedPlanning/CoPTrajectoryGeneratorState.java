package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import javafx.geometry.Side;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.SaveableModuleState;
import us.ihmc.tools.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class CoPTrajectoryGeneratorState extends SaveableModuleState
{
   private final YoPreallocatedList<PlanningFootstep> footsteps;
   private final YoPreallocatedList<PlanningTiming> footstepTimings;
   private final YoPreallocatedList<PlanningShiftFraction> footstepShiftFractions;

   private final YoDouble finalTransferSplitFraction;
   private final YoDouble finalTransferWeightDistribution;
   private final YoDouble finalTransferDuration;
   private final YoDouble percentageStandingWeightDistributionOnLeftFoot;

   private final YoFramePoint2D initialCoP;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSole = new SideDependentList<>();
   private final SideDependentList<FixedFramePose3DBasics> footPoses = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> soleContactFrames = new SideDependentList<>();

   public CoPTrajectoryGeneratorState(YoRegistry registry)
   {
      footsteps = new YoPreallocatedList<>(PlanningFootstep.class, () -> createFootstep(registry), "footstep", registry, 3);
      footstepTimings = new YoPreallocatedList<>(PlanningTiming.class, () -> createTiming(registry), "footstepTiming", registry, 3);
      footstepShiftFractions = new YoPreallocatedList<>(PlanningShiftFraction.class, () -> createShiftFractions(registry), "footstepShiftFraction", registry, 3);
      registerIntegerToSave(footsteps.getYoPosition());
      registerIntegerToSave(footstepTimings.getYoPosition());
      registerIntegerToSave(footstepShiftFractions.getYoPosition());

      finalTransferSplitFraction = new YoDouble("finalTransferSplitFraction", registry);
      finalTransferWeightDistribution = new YoDouble("finalTransferWeightDistribution", registry);
      finalTransferDuration = new YoDouble("finalTransferDuration", registry);
      percentageStandingWeightDistributionOnLeftFoot = new YoDouble("percentageStandingWeightDistributionOnLeftFoot", registry);
      registerDoubleToSave(finalTransferSplitFraction);
      registerDoubleToSave(finalTransferWeightDistribution);
      registerDoubleToSave(finalTransferDuration);
      registerDoubleToSave(percentageStandingWeightDistributionOnLeftFoot);

      percentageStandingWeightDistributionOnLeftFoot.set(0.5);

      initialCoP = new YoFramePoint2D("initialCoP", ReferenceFrame.getWorldFrame(), registry);
      SaveableModuleStateTools.registerYoTuple2DToSave(initialCoP, this);

      for (RobotSide robotSide : RobotSide.values)
      {


         YoFramePose3D footPose = new YoFramePose3D(robotSide.getCamelCaseName() + "FootPose", ReferenceFrame.getWorldFrame(), registry);
         SaveableModuleStateTools.registerYoFramePose3DToSave(footPose, this);
         footPoses.put(robotSide, footPose);

         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getCamelCaseName() + "SoleFrame", footPose);
         soleContactFrames.put(robotSide, soleFrame);

         List<YoFramePoint2D> vertexBuffer = new ArrayList<>();
         String prefix = robotSide.getCamelCaseName() + "FootPolygonInSole";
         for (int i = 0; i < 6; i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "_" + i, soleFrame, registry);
            SaveableModuleStateTools.registerYoTuple2DToSave(vertex, this);
            vertexBuffer.add(vertex);
         }
         YoInteger numberOfVertices = new YoInteger(prefix + "NumVertices", registry);
         registerIntegerToSave(numberOfVertices);
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
      soleContactFrames.get(robotSide).setPoseAndUpdate(footPoses.get(robotSide));
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

   public PlanningFootstep getFootstep(int index)
   {
      return footsteps.get(index);
   }

   public PlanningTiming getTiming(int index)
   {
      return footstepTimings.get(index);
   }

   public PlanningShiftFraction getShiftFraction(int index)
   {
      return footstepShiftFractions.get(index);
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

   public double getFinalTransferSplitFraction()
   {
      return finalTransferSplitFraction.getDoubleValue();
   }

   public double getFinalTransferWeightDistribution()
   {
      return finalTransferWeightDistribution.getDoubleValue();
   }

   public double getPercentageStandingWeightDistributionOnLeftFoot()
   {
      return percentageStandingWeightDistributionOnLeftFoot.getValue();
   }

   public void setFinalTransferDuration(double transferDuration)
   {
      finalTransferDuration.set(transferDuration);
   }

   public void setFinalTransferSplitFraction(double finalTransferSplitFraction)
   {
      this.finalTransferSplitFraction.set(finalTransferSplitFraction);
   }

   public void setFinalTransferWeightDistribution(double finalTransferWeightDistribution)
   {
      this.finalTransferWeightDistribution.set(finalTransferWeightDistribution);
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
      for (int i = 0; i < footstepShiftFractions.size(); i++)
         footstepShiftFractions.get(i).clear();
      footsteps.clear();
      footstepTimings.clear();
      footstepShiftFractions.clear();
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

   public void addFootstepShiftFractions(FootstepShiftFractions shiftFraction)
   {
      if (footstepShiftFractions.size() < footstepShiftFractions.capacity())
         footstepShiftFractions.add().set(shiftFraction);
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

   private PlanningFootstep createFootstep(YoRegistry registry)
   {
      PlanningFootstep footstep = new PlanningFootstep("" + footstepCounter++, registry);
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
