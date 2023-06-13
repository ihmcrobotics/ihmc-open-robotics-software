package us.ihmc.footstepPlanning.tools;

import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepChecker;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;

import java.util.*;

public class FootstepPlannerRejectionReasonReport
{
   private final HashMap<BipedalFootstepPlannerNodeRejectionReason, MutableInt> rejectionReasons = new HashMap<>();
   private final TreeSet<BipedalFootstepPlannerNodeRejectionReason> sortedReasons
         = new TreeSet<>(Comparator.comparing(o -> -rejectionReasons.get(o).getValue()));
   private final Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap;
   private final List<FootstepPlannerIterationData> iterationDataList;
   private long totalNumberOfRejections = 0;
   private final int rejectionReasonIndex;

   public FootstepPlannerRejectionReasonReport(FootstepPlanningModule footstepPlanningModule)
   {
      this(footstepPlanningModule.getEdgeDataMap(), footstepPlanningModule.getIterationData(), footstepPlanningModule.getFootstepPlanVariableDescriptors());
   }

   public FootstepPlannerRejectionReasonReport(FootstepPlannerLog footstepPlanningLog)
   {
      this(footstepPlanningLog.getEdgeDataMap(), footstepPlanningLog.getIterationData(), footstepPlanningLog.getVariableDescriptors());
   }

   public FootstepPlannerRejectionReasonReport(Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap,
                                               List<FootstepPlannerIterationData> iterationDataList,
                                               List<VariableDescriptor> variableDescriptors)
   {
      this.edgeDataMap = edgeDataMap;
      this.iterationDataList = iterationDataList;
      VariableDescriptor rejectionReasonVariableDescriptor = variableDescriptors.stream()
                                                                                .filter(v -> v.getName()
                                                                                              .equalsIgnoreCase(FootstepChecker.rejectionReasonVariable))
                                                                                .findFirst()
                                                                                .get();
      rejectionReasonIndex = variableDescriptors.indexOf(rejectionReasonVariableDescriptor);
   }

   public void update()
   {
      rejectionReasons.clear();
      totalNumberOfRejections = 0;

      for (int i = 0; i < iterationDataList.size(); i++)
      {
         FootstepPlannerIterationData iterationData = iterationDataList.get(i);

         for (int j = 0; j < iterationData.getChildNodes().size(); j++)
         {
            GraphEdge<FootstepGraphNode> edgeKey = new GraphEdge<>(iterationData.getParentNode(), iterationData.getChildNodes().get(j));
            FootstepPlannerEdgeData edgeData = edgeDataMap.get(edgeKey);

            // Null is a valid value for rejection reason
            BipedalFootstepPlannerNodeRejectionReason rejectionReason
                  = BipedalFootstepPlannerNodeRejectionReason.fromByte((byte) edgeData.getDataBuffer()[rejectionReasonIndex]);
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
