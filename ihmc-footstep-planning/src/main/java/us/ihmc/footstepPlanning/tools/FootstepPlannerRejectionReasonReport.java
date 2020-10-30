package us.ihmc.footstepPlanning.tools;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;

import java.util.*;

public class FootstepPlannerRejectionReasonReport
{
   private final FootstepPlanningModule footstepPlanningModule;

   private final HashMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> rejectionReasons = new HashMap<>();
   private final TreeSet<BipedalFootstepPlannerNodeRejectionReason> sortedReasons
         = new TreeSet<>(Comparator.comparing(o -> -rejectionReasons.get(o).getValue()));
   private long totalNumberOfRejections = 0;

   public FootstepPlannerRejectionReasonReport(FootstepPlanningModule footstepPlanningModule)
   {
      this.footstepPlanningModule = footstepPlanningModule;
   }

   public void update()
   {
      rejectionReasons.clear();
      totalNumberOfRejections = 0;

      List<FootstepPlannerIterationData> iterationDataList = footstepPlanningModule.getIterationData();
      for (int i = 0; i < iterationDataList.size(); i++)
      {
         FootstepPlannerIterationData iterationData = iterationDataList.get(i);

         for (int j = 0; j < iterationData.getChildNodes().size(); j++)
         {
            GraphEdge<FootstepGraphNode> edgeKey = new GraphEdge<>(iterationData.getParentNode(), iterationData.getChildNodes().get(j));
            FootstepPlannerEdgeData edgeData = footstepPlanningModule.getEdgeDataMap().get(edgeKey);

            BipedalFootstepPlannerNodeRejectionReason rejectionReason
                  = BipedalFootstepPlannerNodeRejectionReason.fromByte((byte) edgeData.getDataBuffer()[7]);
            rejectionReasons.putIfAbsent(rejectionReason, new MutableInt());
            rejectionReasons.get(rejectionReason).increment();
            totalNumberOfRejections++;
         }
      }

      sortedReasons.clear();
      sortedReasons.addAll(rejectionReasons.keySet());
   }

   public HashMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> getRejectionReasons()
   {
      return rejectionReasons;
   }

   public TreeSet<BipedalFootstepPlannerNodeRejectionReason> getSortedReasons()
   {
      return sortedReasons;
   }

   public long getTotalNumberOfRejections()
   {
      return totalNumberOfRejections;
   }

   public double getRejectionReasonPercentage(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      int numberOfRejections = rejectionReasons.get(rejectionReason).getValue();
      return 100.0 * numberOfRejections / totalNumberOfRejections;
   }
}
