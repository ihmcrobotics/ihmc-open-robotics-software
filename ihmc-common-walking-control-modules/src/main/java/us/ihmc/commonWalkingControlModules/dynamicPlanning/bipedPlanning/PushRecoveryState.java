package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
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
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class PushRecoveryState extends YoSaveableModuleState
{
   private final YoFramePoint2D icpAtStartOfState;

   private final YoPreallocatedList<DynamicPlanningFootstep> footsteps;
   private final YoPreallocatedList<PlanningTiming> footstepTimings;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> footPolygonsInSole = new SideDependentList<>();
   private final SideDependentList<FixedFramePose3DBasics> footPoses = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> soleContactFrames = new SideDependentList<>();

   public PushRecoveryState(YoRegistry registry)
   {
      this(registry, 3);
   }

   public PushRecoveryState(YoRegistry registry, int maxNumberOfStepsToConsider)
   {
      footsteps = new YoPreallocatedList<>(DynamicPlanningFootstep.class, () -> createFootstep(registry), "footstep", registry, maxNumberOfStepsToConsider);
      footstepTimings = new YoPreallocatedList<>(PlanningTiming.class, () -> createTiming(registry), "footstepTiming", registry, maxNumberOfStepsToConsider);
      registerVariableToSave(footsteps.getYoPosition());
      registerVariableToSave(footstepTimings.getYoPosition());

      icpAtStartOfState = new YoFramePoint2D("icpAtStartOfState", ReferenceFrame.getWorldFrame(), registry);

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

      YoSaveableModuleStateTools.registerYoTuple2DToSave(icpAtStartOfState, this);
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

   public FramePose3DReadOnly getFootPose(RobotSide robotSide)
   {
      return footPoses.get(robotSide);
   }

   public FrameConvexPolygon2DReadOnly getFootPolygonInSole(RobotSide robotSide)
   {
      return footPolygonsInSole.get(robotSide);
   }

   public int getNumberOfFootsteps()
   {
      return footsteps.size();
   }

   public FramePoint2DReadOnly getIcpAtStartOfState()
   {
      return icpAtStartOfState;
   }

   public DynamicPlanningFootstep getFootstep(int index)
   {
      return footsteps.get(index);
   }

   public PlanningTiming getTiming(int index)
   {
      return footstepTimings.get(index);
   }

}
