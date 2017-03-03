package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class YoGraphicAbstractShape extends YoGraphic
{
   protected final YoFramePoint yoFramePoint;
   protected final YoFrameOrientation yoFrameOrientation;
   protected final double scale;
   private final Vector3D translationVector = new Vector3D();
   private final Point3D tempPoint = new Point3D();
   private final Quaternion tempQuaternion = new Quaternion();

   protected YoGraphicAbstractShape(String name, YoFramePoint framePoint, YoFrameOrientation frameOrientation, double scale)
   {
      super(name);
      framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      this.yoFramePoint = framePoint;
      this.yoFrameOrientation = frameOrientation;

      this.scale = scale;
   }

   public void setPose(FramePose framePose)
   {
      yoFramePoint.checkReferenceFrameMatch(framePose.getReferenceFrame());

      framePose.getPosition(tempPoint);
      yoFramePoint.setPoint(tempPoint);

      framePose.getOrientation(tempQuaternion);
      yoFrameOrientation.set(tempQuaternion);
   }

   public void setPosition(double x, double y, double z)
   {
      yoFramePoint.set(x, y, z);
   }

   public void getPosition(FramePoint framePointToPack)
   {
      yoFramePoint.getFrameTuple(framePointToPack);
   }

   public void setPosition(FramePoint position)
   {
      yoFramePoint.set(position);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      this.yoFrameOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      this.yoFrameOrientation.set(orientation);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      this.yoFrameOrientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      Vector3D translationToWorld = new Vector3D();

      transformToWorld.getTranslation(translationToWorld);

      this.yoFramePoint.set(translationToWorld);
      FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), transformToWorld);

      double[] yawPitchRoll = orientation.getYawPitchRoll();
      yoFrameOrientation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
         throw new RuntimeException("referenceFrame == null");

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ReferenceFrame ancestorFrame = referenceFrame;

      // March up the parents until you get to the world:
      while (!ancestorFrame.isWorldFrame())
      {
         RigidBodyTransform transformToAncestor = ancestorFrame.getTransformToParent();

         RigidBodyTransform tempTransform3D = new RigidBodyTransform(transformToAncestor);
         tempTransform3D.multiply(transformToWorld);

         transformToWorld = tempTransform3D;

         ReferenceFrame newAncestorFrame = ancestorFrame.getParent();

         if (newAncestorFrame == null)
            throw new RuntimeException("No ancestor path to world. referenceFrame = " + referenceFrame + ", most ancient = " + ancestorFrame);

         ancestorFrame = newAncestorFrame;
      }

      setTransformToWorld(transformToWorld);
   }

   private Vector3D rotationEulerVector = new Vector3D();

   @Override
   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      transform3D.setIdentity();
      translationVector.set(yoFramePoint.getX(), yoFramePoint.getY(), yoFramePoint.getZ());
      yoFrameOrientation.getEulerAngles(rotationEulerVector);

      transform3D.setRotationEuler(rotationEulerVector);
      transform3D.setTranslation(translationVector);
      transform3D.setScale(scale);
   }

   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   public void setPoseToNaN()
   {
      yoFramePoint.setToNaN();
      yoFrameOrientation.setToNaN();
   }

   @Override
   protected boolean containsNaN()
   {
      if (yoFramePoint.containsNaN())
         return true;
      if (yoFrameOrientation.containsNaN())
         return true;

      return false;
   }
}