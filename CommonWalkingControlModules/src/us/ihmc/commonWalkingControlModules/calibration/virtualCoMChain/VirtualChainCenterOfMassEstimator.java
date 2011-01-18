package us.ihmc.commonWalkingControlModules.calibration.virtualCoMChain;

import java.util.ArrayList;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class VirtualChainCenterOfMassEstimator
{
   private final ReferenceFrame baseFrame;

   private final ArrayList<FrameVector>  virtualMassParameterVectors;

   public VirtualChainCenterOfMassEstimator(ReferenceFrame baseFrame, ArrayList<FrameVector> virtualMassParameterVectors)
   {
      this.baseFrame = baseFrame;
      this.virtualMassParameterVectors = virtualMassParameterVectors;
   }

   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());

   public FramePoint computeCenterOfMass()
   {
      FramePoint ret = new FramePoint(baseFrame);

      for (int i = 0; i < virtualMassParameterVectors.size(); i++)
      {
         tempFrameVector.set(virtualMassParameterVectors.get(i));
//         System.out.println("tempFrameVector before changeFrame= "  + tempFrameVector);
         
         tempFrameVector.changeFrame(baseFrame);
//         System.out.println("tempFrameVector after changeFrame= "  + tempFrameVector);

         ret.add(tempFrameVector);
      }

//      System.out.println();

      
      return ret;
   }
   
   public String toString()
   {
      String ret = "";
      
      for (FrameVector frameVector : virtualMassParameterVectors)
      {
         ret = ret + frameVector + "\n";
      }
      
      return ret;
   }
}
