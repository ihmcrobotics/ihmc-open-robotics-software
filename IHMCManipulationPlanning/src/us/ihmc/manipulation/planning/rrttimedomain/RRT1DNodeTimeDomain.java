package us.ihmc.manipulation.planning.rrttimedomain;

import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelPoseValidityTester;
import us.ihmc.manipulation.planning.rrt.RRTNode;

public class RRT1DNodeTimeDomain extends RRTNode
{
   public static SolarPanelPoseValidityTester nodeValidityTester;
   
   /*
    * getNodeData(0) = time
    * getNodeData(1) = pelvisYaw
    */
   
   protected double maximumVelocityOfStep;
   protected double maximumDisplacementOfStep;
   
   public RRT1DNodeTimeDomain()
   {
      super(2);
   }
   
   public RRT1DNodeTimeDomain(double timeK, double pelvisYawK)
   {
      super(2);
      super.setNodeData(0, timeK);
      super.setNodeData(1, pelvisYawK);
   }
   
   public void setMaximumVelocityOfStep(double maximumVelocityOfStep)
   {
      this.maximumVelocityOfStep = maximumVelocityOfStep;
   }
   
   public void setmaximumDisplacementOfStep(double maximumDisplacementOfStep)
   {
      this.maximumDisplacementOfStep = maximumDisplacementOfStep;
   }

   @Override
   public boolean isValidNode()
   {      
      nodeValidityTester.setTemporaryNodeData(getNodeData(0));
      return nodeValidityTester.isValid;
   }

   @Override
   public RRTNode createNode()
   {
      // TODO Auto-generated method stub
      return null;
   }
}
