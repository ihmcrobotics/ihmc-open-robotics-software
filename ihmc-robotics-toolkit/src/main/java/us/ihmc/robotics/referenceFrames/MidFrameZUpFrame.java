package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class MidFrameZUpFrame extends ReferenceFrame
{
   private final ReferenceFrame parentZUpFrame, frameOne, frameTwo;

   private final FramePoint3D originOneInParent;
   private final FramePoint3D originTwoInParent;
   private final FrameVector3D vectorBetweenFrames;
   private final Vector2D vectorBetweenFrames2d = new Vector2D();
   private final Vector3D parentToMidpointVector3d = new Vector3D();
   private final RigidBodyTransform transform3D = new RigidBodyTransform();

   public MidFrameZUpFrame(String name, ReferenceFrame parentZUpFrame, ReferenceFrame frameOne, ReferenceFrame frameTwo)
   {
      super(name, parentZUpFrame, false, true);

      if (!parentZUpFrame.isZupFrame())
      {
         throw new RuntimeException("!parentZUpFrame.isZupFrame()");
      }

      this.parentZUpFrame = parentZUpFrame;
      this.frameOne = frameOne;
      this.frameTwo = frameTwo;

      originOneInParent = new FramePoint3D(frameOne);
      originTwoInParent = new FramePoint3D(frameTwo);
      vectorBetweenFrames = new FrameVector3D(parentZUpFrame);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      // Everything is based on the feet positions in the body frame:
      originOneInParent.setToZero(frameOne);
      originOneInParent.changeFrame(parentZUpFrame);

      originTwoInParent.setToZero(frameTwo);
      originTwoInParent.changeFrame(parentZUpFrame);

      // Rotation is to yaw onto the line between the feet:

      vectorBetweenFrames.sub(originTwoInParent, originOneInParent);

      vectorBetweenFrames2d.set(vectorBetweenFrames.getX(), vectorBetweenFrames.getY());


      // Translation is average between the two feet:
      parentToMidpointVector3d.add(originOneInParent, originTwoInParent);
      parentToMidpointVector3d.scale(0.5);

      // Create the transform:
      transform3D.setIdentity();

      if (vectorBetweenFrames2d.lengthSquared() < 1e-7)
         return;

      vectorBetweenFrames2d.normalize();
      transform3D.setRotationYawAndZeroTranslation(Math.PI / 2.0 + Math.atan2(vectorBetweenFrames2d.getY(), vectorBetweenFrames2d.getX()));

      transform3D.setTranslation(parentToMidpointVector3d);

      transformToParent.set(transform3D);
   }
}