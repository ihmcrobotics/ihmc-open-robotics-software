package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.saveableModule.SaveableModuleState;
import us.ihmc.robotics.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class CoPTrajectoryGeneratorState extends SaveableModuleState
{
   private final PreallocatedList<PlanningFootstep> footsteps;
   private final PreallocatedList<PlanningTiming> footstepTimings;
   private final PreallocatedList<PlanningShiftFraction> footstepShiftFractions;

   private final YoFramePoint2D initialCoP;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSole = new SideDependentList<>();
   private final SideDependentList<FixedFramePose3DBasics> footPoses = new SideDependentList<>();

   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   public CoPTrajectoryGeneratorState(SideDependentList<? extends ReferenceFrame> soleFrames, YoRegistry registry)
   {
      this.soleFrames = soleFrames;

      footsteps = new PreallocatedList<>(PlanningFootstep.class, () -> createFootstep(registry), 3);
      footstepTimings = new PreallocatedList<>(PlanningTiming.class, () -> createTiming(registry), 3);
      footstepShiftFractions = new PreallocatedList<>(PlanningShiftFraction.class, () -> createShiftFractions(registry), 3);

      initialCoP = new YoFramePoint2D("initialCoP", ReferenceFrame.getWorldFrame(), registry);
      SaveableModuleStateTools.registerYoTuple2DToSave(initialCoP, this);

      for (RobotSide robotSide : RobotSide.values)
      {
         List<YoFramePoint2D> vertexBuffer = new ArrayList<>();
         String prefix = robotSide.getCamelCaseName() + "FootPolygonInSole";
         for (int i = 0; i < 6; i++)
         {
            YoFramePoint2D vertex = new YoFramePoint2D(prefix + "_" + i, soleFrames.get(robotSide), registry);
            SaveableModuleStateTools.registerYoTuple2DToSave(vertex, this);
            vertexBuffer.add(vertex);
         }
         YoInteger numberOfVertices = new YoInteger(prefix + "NumVertices", registry);
         registerIntegerToSave(numberOfVertices);
         YoFrameConvexPolygon2D footPolygonInSole = new YoFrameConvexPolygon2D(vertexBuffer,
                                                                               numberOfVertices,
                                                                               soleFrames.get(robotSide));
         footPolygonsInSole.put(robotSide, footPolygonInSole);

         YoFramePose3D footPose = new YoFramePose3D(robotSide.getCamelCaseName() + "FootPose", ReferenceFrame.getWorldFrame(), registry);
         SaveableModuleStateTools.registerYoFramePose3DToSave(footPose, this);
         footPoses.put(robotSide, footPose);
      }
   }

   public void initializeStance(SideDependentList<? extends FrameConvexPolygon2DReadOnly> feetInSoleZUpFrames)
   {
      for (RobotSide robotSide : RobotSide.values)
         initializeStance(robotSide, feetInSoleZUpFrames.get(robotSide));
   }

   public void initializeStance(RobotSide robotSide, FrameConvexPolygon2DReadOnly supportPolygon)
   {
      footPolygonsInSole.get(robotSide).setMatchingFrame(supportPolygon, false);
      footPoses.get(robotSide).setFromReferenceFrame(soleFrames.get(robotSide));
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
      initialCoP.setToNaN();
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

   private PlanningFootstep createFootstep(YoRegistry registry)
   {
      PlanningFootstep footstep = new PlanningFootstep("" + footsteps.size(), registry);
      registerStateToSave(footstep);
      return footstep;
   }

   private PlanningTiming createTiming(YoRegistry registry)
   {
      PlanningTiming timing = new PlanningTiming("" + footstepTimings.size(), registry);
      registerStateToSave(timing);
      return timing;
   }

   private PlanningShiftFraction createShiftFractions(YoRegistry registry)
   {
      PlanningShiftFraction shiftFractions = new PlanningShiftFraction("" + footstepShiftFractions.size(), registry);
      registerStateToSave(shiftFractions);
      return shiftFractions;
   }
}
