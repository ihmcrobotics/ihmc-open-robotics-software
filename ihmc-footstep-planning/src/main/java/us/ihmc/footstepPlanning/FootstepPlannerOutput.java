package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;

public class FootstepPlannerOutput
{
   private int planId;
   private final FootstepPlan footstepPlan = new FootstepPlan();
   private FootstepPlanningResult result;
   private PlanarRegionsList planarRegionsList;
   private final ArrayList<Pose3D> bodyPath = new ArrayList<>();
   private final Pose3D lowLevelGoal = new Pose3D();

   public void clear()
   {
      planId = -1;
      footstepPlan.clear();
      result = null;
      planarRegionsList = null;
      bodyPath.clear();
      lowLevelGoal.setToNaN();
   }

   public int getPlanId()
   {
      return planId;
   }

   public FootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public FootstepPlanningResult getResult()
   {
      return result;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public ArrayList<Pose3D> getBodyPath()
   {
      return bodyPath;
   }

   public Pose3D getLowLevelGoal()
   {
      return lowLevelGoal;
   }

   public void setPlanId(int planId)
   {
      this.planId = planId;
   }

   public void setResult(FootstepPlanningResult result)
   {
      this.result = result;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setLowLevelGoal(Pose3D lowLevelGoal)
   {
      this.lowLevelGoal.set(lowLevelGoal);
   }

   public void setPacket(FootstepPlanningToolboxOutputStatus outputStatus)
   {
      outputStatus.setPlanId(getPlanId());
      outputStatus.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(getFootstepPlan(), -1.0, -1.0, ExecutionMode.OVERRIDE));
      outputStatus.setFootstepPlanningResult(getResult().toByte());
      outputStatus.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(getPlanarRegionsList()));
      outputStatus.getBodyPath().clear();
      outputStatus.getLowLevelPlannerGoal().set(getLowLevelGoal());

      for (int i = 0; i < bodyPath.size(); i++)
      {
         outputStatus.getBodyPath().add().set(bodyPath.get(i));
      }
   }
}
