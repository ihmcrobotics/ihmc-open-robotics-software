package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.saveableModule.YoSaveableModuleState;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

public class DynamicPlanningFootstep extends YoSaveableModuleState
{
   private final YoPreallocatedList<YoPoint2D> predictedContactPoints;
   private final YoFramePose3D footstepPose;
   private final YoEnum<RobotSide> stepSide;

   public DynamicPlanningFootstep(String suffix, YoRegistry registry)
   {
      footstepPose = new YoFramePose3D("footstepPose" + suffix, ReferenceFrame.getWorldFrame(), registry);
      YoSaveableModuleStateTools.registerYoFramePose3DToSave(footstepPose, this);
      predictedContactPoints = new YoPreallocatedList<>(YoPoint2D.class, () -> createYoContactPoint(suffix, registry), "footstep" + suffix + "ContactPoint", registry, 6);
      registerVariableToSave(predictedContactPoints.getYoPosition());
      stepSide = new YoEnum<>("stepSide" + suffix, registry, RobotSide.class);
      registerVariableToSave(stepSide);

      clear();
   }

   private int contactPointCounter = 0;

   private YoPoint2D createYoContactPoint(String suffix, YoRegistry registry)
   {
      YoPoint2D point = new YoPoint2D("footstep" + suffix + "ContactPoint" + contactPointCounter++, registry);
      YoSaveableModuleStateTools.registerYoTuple2DToSave(point, this);
      return point;
   }

   public FramePose3DReadOnly getFootstepPose()
   {
      return footstepPose;
   }

   public RobotSide getRobotSide()
   {
      return stepSide.getEnumValue();
   }

   public boolean hasPredictedContactPoints()
   {
      return !predictedContactPoints.isEmpty();
   }

   public List<? extends Point2DReadOnly> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   public void clear()
   {
      footstepPose.setToNaN();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         predictedContactPoints.get(i).setToNaN();
      predictedContactPoints.clear();
   }

   public void set(Footstep footstep)
   {
      footstepPose.set(footstep.getFootstepPose());
      stepSide.set(footstep.getRobotSide());
      if (!footstep.hasPredictedContactPoints())
         return;

      for (int i = 0; i < Math.min(footstep.getPredictedContactPoints().size(), predictedContactPoints.capacity()); i++)
         predictedContactPoints.add().set(footstep.getPredictedContactPoints().get(i));
   }
}
