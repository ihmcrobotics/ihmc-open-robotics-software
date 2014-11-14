package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import us.ihmc.utilities.screwTheory.RevoluteJointReferenceFrame;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;

public class VirtualLinkFromPinJoint extends VirtualLinkFromJoint
{

   
   public VirtualLinkFromPinJoint(PinJoint joint)
   {
      super(joint);
   }

   public void updateReferenceFrameFromJointAngle()
   {
      RevoluteJointReferenceFrame referenceFrame = (RevoluteJointReferenceFrame) virtualLinkFrameVector.getReferenceFrame();
      
      if (this.referenceFrame != referenceFrame)
      {
         throw new RuntimeException("(this.referenceFrame != referenceFrame)");
      }
      referenceFrame.setAndUpdate(((OneDegreeOfFreedomJoint) joint).getQ().getDoubleValue());
      referenceFrame.update();
   }
}
