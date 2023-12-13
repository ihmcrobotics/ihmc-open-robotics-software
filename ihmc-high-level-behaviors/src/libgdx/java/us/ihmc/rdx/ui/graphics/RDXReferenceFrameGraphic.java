package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXReferenceFrameGraphic extends RDXModelInstance
{
   private final RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
   private final FramePose3D framePose3D = new FramePose3D();
   private ReferenceFrame referenceFrame;

   public RDXReferenceFrameGraphic(double length)
   {
      super(RDXModelBuilder.createCoordinateFrameInstance(length));
   }

   public RDXReferenceFrameGraphic(double length, Color color)
   {
      super(RDXModelBuilder.createCoordinateFrameInstance(length, color));
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      updateFromLastGivenFrame();
   }

   public void updateFromLastGivenFrame()
   {
      framePose3D.setToZero(referenceFrame);
      updateFromFramePose();
   }

   public FramePose3D getFramePose3D()
   {
      return framePose3D;
   }

   public void updateFromFramePose()
   {
      framePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      framePose3D.get(rigidBodyTransform);
      LibGDXTools.toLibGDX(rigidBodyTransform, transform);
   }

   public void updateFromFramePose(FramePose3D framePose3D)
   {
      referenceFrame = framePose3D.getReferenceFrame();
      this.framePose3D.set(framePose3D);
      updateFromFramePose();
   }

   public void dispose()
   {
      model.dispose();
   }
}
