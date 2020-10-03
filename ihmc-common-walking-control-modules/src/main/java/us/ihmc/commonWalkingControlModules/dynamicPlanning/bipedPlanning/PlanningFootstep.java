package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.saveableModule.SaveableModuleState;
import us.ihmc.robotics.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

public class PlanningFootstep extends SaveableModuleState
{
   private final PreallocatedList<YoPoint2D> predictedContactPoints;
   private final YoFramePose3D footstepPose;
   private final YoEnum<RobotSide> stepSide;

   public PlanningFootstep(String suffix, YoRegistry registry)
   {
      footstepPose = new YoFramePose3D("footstepPose" + suffix, ReferenceFrame.getWorldFrame(), registry);
      SaveableModuleStateTools.registerYoFramePose3DToSave(footstepPose, this);
      predictedContactPoints = new PreallocatedList<>(YoPoint2D.class, () -> createYoContactPoint(suffix, registry), 6);
      stepSide = new YoEnum<>("stepSide" + suffix, registry, RobotSide.class);
      registerEnumToSave(stepSide);

      clear();
   }

   private int contactPointCounter = 0;

   private YoPoint2D createYoContactPoint(String suffix, YoRegistry registry)
   {
      YoPoint2D point = new YoPoint2D("footstep" + suffix + "ContactPoint" + contactPointCounter++, registry);
      SaveableModuleStateTools.registerYoTuple2DToSave(point, this);
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
      for (int i = 0; i < Math.min(footstep.getPredictedContactPoints().size(), predictedContactPoints.capacity()); i++)
         predictedContactPoints.add().set(footstep.getPredictedContactPoints().get(i));
   }
}
