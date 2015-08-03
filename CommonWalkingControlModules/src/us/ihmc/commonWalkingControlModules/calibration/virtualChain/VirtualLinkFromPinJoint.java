package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.robotics.screwTheory.RevoluteJointReferenceFrame;

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
