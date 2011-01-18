package us.ihmc.commonWalkingControlModules.calibration.virtualCoMChain;

import us.ihmc.utilities.screwTheory.RevoluteJointReferenceFrame;

import com.yobotics.simulationconstructionset.PinJoint;

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
      referenceFrame.set(((PinJoint) joint).getQ().getDoubleValue());
      referenceFrame.update();
   }
}
