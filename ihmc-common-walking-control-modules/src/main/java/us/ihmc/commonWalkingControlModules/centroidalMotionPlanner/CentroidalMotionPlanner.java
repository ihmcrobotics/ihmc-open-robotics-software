package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

public class CentroidalMotionPlanner
{
   private final double mass;
   private final double Ixx;
   private final double Iyy;
   private final double Izz;
   private final double deltaTMin;

   public CentroidalMotionPlanner(double mass, double nominalIxx, double nominalIyy, double nominalIzz, double deltaTMin)
   {
      this.mass = mass;
      this.Ixx = nominalIxx;
      this.Iyy = nominalIyy;
      this.Izz = nominalIzz;
      this.deltaTMin = deltaTMin;
   }

   public void sumbitNode(CentroidalMotionNode dataNode)
   {
//      double nodeTime = dataNode.getTime();
//      int indexToAdd = 0;
//      for (indexToAdd = 0; indexToAdd < nodeList.size(); indexToAdd++)
//      {
//         CentroidalMotionNode listNode = nodeList.get(indexToAdd);
//         if (nodeTime < listNode.getTime() + deltaTMin)
//            break;
//      }
//      if (indexToAdd == nodeList.size())
//         nodeList.add().set(dataNode);
//      else if (nodeTime > nodeList.get(indexToAdd).getTime() + deltaTMin)
//         nodeList.add(indexToAdd, dataNode);
//      else
//         mergeNodes(dataNode, nodeList.get(indexToAdd));
   }

   private void mergeNodes(CentroidalMotionNode nodeToDiscard, CentroidalMotionNode nodeToKeep)
   {
      throw new RuntimeException("Unimplmented code");
   }
}
