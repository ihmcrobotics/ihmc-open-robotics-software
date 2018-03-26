package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CenterOfMassCalculator
{
   private final ReferenceFrame desiredFrame;

   private final RigidBody[] rigidBodies;
   private double totalMass;
   private final FramePoint3D centerOfMassPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FramePoint3D tempPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

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
      centerOfMassPosition.setToZero(desiredFrame);
      totalMass = 0.0;

      for (RigidBody rigidBody : rigidBodies)
      {
         rigidBody.getCoMOffset(tempPoint);
         double mass = rigidBody.getInertia().getMass();
         tempPoint.changeFrame(desiredFrame);
         tempPoint.scale(mass);
         centerOfMassPosition.add(tempPoint);
         totalMass += mass;
      }

      centerOfMassPosition.scale(1.0 / totalMass);
   }
   
   public FramePoint3D getCenterOfMass()
   {
      return new FramePoint3D(centerOfMassPosition);
   }

   public void getCenterOfMass(FramePoint3D centerOfMassToPack)
   {
      centerOfMassToPack.setIncludingFrame(this.centerOfMassPosition);
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
