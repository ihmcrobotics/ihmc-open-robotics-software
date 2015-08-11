package us.ihmc.robotics.referenceFrames;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;

import javax.vecmath.Vector3d;

public class CenterOfMassReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 97944957592733289L;
   private final CenterOfMassCalculator centerOfMassCalculator;
   private final FramePoint centerOfMass;
   private final Vector3d centerOfMassVector3d = new Vector3d();

   public CenterOfMassReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBody rootBody)
   {
      super(frameName, parentFrame);
      centerOfMassCalculator = new CenterOfMassCalculator(rootBody, parentFrame);
      centerOfMass = new FramePoint(parentFrame);
   }

   public CenterOfMassReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBody[] rigidBodies)
   {
      super(frameName, parentFrame);
      centerOfMassCalculator = new CenterOfMassCalculator(rigidBodies, parentFrame);
      centerOfMass = new FramePoint(parentFrame);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      centerOfMassCalculator.compute();
      centerOfMassCalculator.packCenterOfMass(centerOfMass);
      centerOfMassVector3d.set(centerOfMass.getPoint());
      transformToParent.setIdentity();
      transformToParent.setTranslation(centerOfMassVector3d);
   }

}
