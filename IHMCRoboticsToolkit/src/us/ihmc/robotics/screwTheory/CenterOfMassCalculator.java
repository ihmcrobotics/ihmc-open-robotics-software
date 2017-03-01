package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CenterOfMassCalculator
{
   private final ReferenceFrame desiredFrame;

   private final RigidBody[] rigidBodies;
   private double totalMass;
   private final FramePoint centerOfMass = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint tempPoint = new FramePoint(ReferenceFrame.getWorldFrame());

   public CenterOfMassCalculator(RigidBody rootBody, ReferenceFrame desiredFrame)
   {
      this(ScrewTools.computeSupportAndSubtreeSuccessors(rootBody), desiredFrame); //TODO: This gets too much stuff. Shouldn't it just get the rootBody and everything posterior to it?

//    this(ScrewTools.computeSubtreeSuccessors(rootBody.getParentJoint()), desiredFrame); //TODO: Should it be something like this instead?
   }

   public CenterOfMassCalculator(RigidBody[] rigidBodies, ReferenceFrame desiredFrame)
   {
      this.rigidBodies = rigidBodies;
      this.desiredFrame = desiredFrame;
   }
   
   public CenterOfMassCalculator(ArrayList<RigidBody> rigidBodies, ReferenceFrame desiredFrame)
   {
      this.rigidBodies = new RigidBody[rigidBodies.size()];
      rigidBodies.toArray(this.rigidBodies);
      this.desiredFrame = desiredFrame;
   }


   public void compute()
   {
      centerOfMass.setToZero(desiredFrame);
      totalMass = 0.0;

      for (RigidBody rigidBody : rigidBodies)
      {
         rigidBody.getCoMOffset(tempPoint);
         double mass = rigidBody.getInertia().getMass();
         tempPoint.changeFrame(desiredFrame);
         tempPoint.scale(mass);
         centerOfMass.add(tempPoint);
         totalMass += mass;
      }

      centerOfMass.scale(1.0 / totalMass);
   }

   public FramePoint getCenterOfMass()
   {
      return new FramePoint(centerOfMass);
   }

   public void getCenterOfMass(FramePoint centerOfMassToPack)
   {
      centerOfMassToPack.setIncludingFrame(this.centerOfMass);
   }

   public double getTotalMass()
   {
      return totalMass;
   }

   public ReferenceFrame getDesiredFrame()
   {
      return desiredFrame;
   }
}
