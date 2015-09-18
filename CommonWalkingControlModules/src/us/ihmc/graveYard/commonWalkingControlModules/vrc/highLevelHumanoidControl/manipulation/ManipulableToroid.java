package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class ManipulableToroid
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final RigidBody toroid;
   private final TorusLocatorFrame toroidBeforeJointFrame;

   private final DoubleYoVariable toroidQ = new DoubleYoVariable("toroidQ", registry);
   private final DoubleYoVariable toroidQd = new DoubleYoVariable("toroidQd", registry);
   private final DoubleYoVariable toroidRadius = new DoubleYoVariable("toroidRadius", registry);

   private final RevoluteJoint toroidJoint;
   private final DoubleYoVariable damping = new DoubleYoVariable("toroidDamping", registry);

   private final List<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();

   public ManipulableToroid(String namePrefix, RigidBody toroidBase,
                            YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      toroidBeforeJointFrame = new TorusLocatorFrame(namePrefix + "ToroidLocatorFrame", toroidBase.getBodyFixedFrame());
      toroidBeforeJointFrame.update();
      
      FrameVector toroidJointAxis = new FrameVector(toroidBeforeJointFrame, 0.0, 0.0, 1.0);
      toroidJoint = new RevoluteJoint("toroidJoint", toroidBase, toroidBeforeJointFrame, toroidJointAxis);
      toroid = ScrewTools.addRigidBody("toroid", toroidJoint, new Matrix3d(), 0.0, new Vector3d());    // TODO: set mass properties

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicReferenceFrame toroidBeforeJointFrameViz = new YoGraphicReferenceFrame(toroidBeforeJointFrame, registry, 0.3);
         yoGraphicsListRegistry.registerYoGraphic("toroidFrames", toroidBeforeJointFrameViz);
         dynamicGraphicReferenceFrames.add(toroidBeforeJointFrameViz);

         YoGraphicReferenceFrame toroidAfterJointFrameViz = new YoGraphicReferenceFrame(toroid.getParentJoint().getFrameAfterJoint(), registry, 0.1);
         yoGraphicsListRegistry.registerYoGraphic("toroidFrames", toroidAfterJointFrameViz);
         dynamicGraphicReferenceFrames.add(toroidAfterJointFrameViz);
      }

      parentRegistry.addChild(registry);
   }

   public RigidBody getToroid()
   {
      return toroid;
   }

   public ReferenceFrame getStaticToroidReferenceFrame()
   {
      return toroidBeforeJointFrame;
   }

   public OneDoFJoint getToroidJoint()
   {
      return toroidJoint;
   }

   public double getDamping()
   {
      return damping.getDoubleValue();
   }

   public double getToroidRadius()
   {
      return toroidRadius.getDoubleValue();
   }

   public void setToroidRadius(double toroidRadius)
   {
      this.toroidRadius.set(toroidRadius);
   }

   public void setQ(double angle)
   {
      toroidQ.set(angle);
      toroidJoint.setQ(angle);
      toroidJoint.getFrameAfterJoint().update();
   }

   public void setQd(double velocity)
   {
      toroidQd.set(velocity);
      toroidJoint.setQd(velocity);
   }

   public void setToroidLocation(FramePose framePose)
   {
      FramePose copy = new FramePose(framePose);

      Matrix3d rotationMatrix = new Matrix3d();

      framePose.changeFrame(toroidBeforeJointFrame.getParent());
      FramePoint origin = new FramePoint();
      copy.getPositionIncludingFrame(origin);
      FrameVector zAxis = new FrameVector(copy.getReferenceFrame());
      framePose.getOrientation(rotationMatrix);
      rotationMatrix.getColumn(2, zAxis.getVector());
      toroidBeforeJointFrame.set(origin, zAxis);

      copy.changeFrame(toroidBeforeJointFrame);
      copy.getOrientation(rotationMatrix);
      AxisAngle4d axisAngle = new AxisAngle4d();
//      axisAngle.set(rotationMatrix);
      RotationFunctions.axisAngleFromMatrix(rotationMatrix, axisAngle);
      setQ(axisAngle.getAngle());

   }



   public double getQ()
   {
      return toroidQ.getDoubleValue();
   }

   private class TorusLocatorFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 7776724514563190641L;
      private final Vector3d origin = new Vector3d();
      private final Vector3d zAxis = new Vector3d(0.0, 0.0, 1.0);

      private final Vector3d xAxis = new Vector3d();
      private final Vector3d yAxis = new Vector3d();
      private final Matrix3d rotationMatrix = new Matrix3d();

      public TorusLocatorFrame(String frameName, ReferenceFrame parentFrame)
      {
         super(frameName, parentFrame);
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         yAxis.set(0.0, 0.0, 1.0);
         xAxis.cross(yAxis, zAxis);
         double length = xAxis.length();
         if (length > 1e-12)
            xAxis.scale(1.0 / length);
         else
            xAxis.set(1.0, 0.0, 0.0);
         yAxis.cross(zAxis, xAxis);
         yAxis.normalize();

         rotationMatrix.setColumn(0, xAxis);
         rotationMatrix.setColumn(1, yAxis);
         rotationMatrix.setColumn(2, zAxis);

         if (!RotationFunctions.isRotationProper(rotationMatrix))
            throw new RuntimeException("rotation not proper");

         transformToParent.setRotationAndZeroTranslation(rotationMatrix);
         transformToParent.setTranslation(origin);
      }

      public void set(FramePoint origin, FrameVector zAxis)
      {
         origin.changeFrame(getParent());
         zAxis.changeFrame(getParent());

         this.origin.set(origin.getPoint());
         this.zAxis.set(zAxis.getVector());
         this.zAxis.normalize();
         update();
      }

      public void set(RigidBodyTransform toroidToWorldTransform)
      {
         toroidToWorldTransform.getRotation(rotationMatrix);
         rotationMatrix.getColumn(2, zAxis);
         toroidToWorldTransform.get(origin);
         update();
      }
   }
}
