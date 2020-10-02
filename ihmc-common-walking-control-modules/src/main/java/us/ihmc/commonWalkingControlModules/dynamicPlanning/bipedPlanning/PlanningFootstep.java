package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.saveableModule.SaveableModuleState;
import us.ihmc.robotics.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PlanningFootstep extends SaveableModuleState
{
   private final RecyclingArrayList<YoPoint2D> predictedContactPoints;
   private final YoFramePose3D footstepPose;

   public PlanningFootstep(String suffix, YoRegistry registry)
   {
      footstepPose = new YoFramePose3D("footstepPose" + suffix, ReferenceFrame.getWorldFrame(), registry);
      SaveableModuleStateTools.registerYoFramePose3DToSave(footstepPose, this);
      predictedContactPoints = new RecyclingArrayList<>(6, () -> createYoContactPoint(suffix, registry));

      clear();
   }

   private YoPoint2D createYoContactPoint(String suffix, YoRegistry registry)
   {
      YoPoint2D point = new YoPoint2D("footstep" + suffix + "ContactPoint" + predictedContactPoints.size(), registry);
      SaveableModuleStateTools.registerYoTuple2DToSave(point, this);
      return point;
   }

   public void clear()
   {
      footstepPose.setToNaN();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         predictedContactPoints.get(i).setToNaN();
      predictedContactPoints.clear();
   }
}
