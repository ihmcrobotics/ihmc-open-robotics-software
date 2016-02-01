package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.util.ArrayList;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class VirtualChainData
{   
   private final ArrayList<VirtualChainDataPoint> virtualChainDataPoints;
   
   public VirtualChainData()
   {     
      virtualChainDataPoints = new ArrayList<VirtualChainDataPoint>();
   }
   
   public void addData(VirtualChainDataPoint virtualChainDataPoint)
   {
      virtualChainDataPoints.add(virtualChainDataPoint);
   }
   
   public void addData(ReferenceFrame baseFrame, ArrayList<ReferenceFrame> referenceFrames, FramePoint2d centerOfMassProjection)
   {
      VirtualChainDataPoint dataPoint = new VirtualChainDataPoint(baseFrame, referenceFrames, centerOfMassProjection);
      
      virtualChainDataPoints.add(dataPoint);
   }
   
   
   
   public String toString()
   {
      String ret = "";
      for (VirtualChainDataPoint virtualChainDataPoint : virtualChainDataPoints)
      {
           ret = ret + virtualChainDataPoint + "\n";
      }
      
      return ret;
   }

   public int getNumberOfDataPoints()
   {
      return virtualChainDataPoints.size();
   }

   public ArrayList<VirtualChainDataPoint> getVirtualChainDataPoints()
   {
      return virtualChainDataPoints;
   }

   public int getNumberOfDegreesOfFreedom()
   {
      return virtualChainDataPoints.get(0).getNumberOfDegreesOfFreedom();
   }
   
}
