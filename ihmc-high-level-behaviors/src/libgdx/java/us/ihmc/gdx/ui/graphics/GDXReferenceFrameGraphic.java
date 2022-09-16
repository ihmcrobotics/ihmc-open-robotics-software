package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;

public class GDXReferenceFrameGraphic extends GDXModelInstance
{
   private final RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
   private final FramePose3D framePose3D = new FramePose3D();
   private ReferenceFrame referenceFrame;

   public GDXReferenceFrameGraphic(double length)
   {
      super(GDXModelBuilder.createCoordinateFrameInstance(length));
   }

   public GDXReferenceFrameGraphic(double length, Color color)
   {
      super(GDXModelBuilder.createCoordinateFrameInstance(length, color));
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
      GDXTools.toGDX(rigidBodyTransform, transform);
   }

   public void dispose()
   {
      model.dispose();
   }
}
