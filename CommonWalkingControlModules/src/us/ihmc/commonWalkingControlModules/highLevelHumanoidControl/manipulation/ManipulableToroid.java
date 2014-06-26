package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;

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

   private final List<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();

   public ManipulableToroid(String namePrefix, RigidBody toroidBase,
                            DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      toroidBeforeJointFrame = new TorusLocatorFrame(namePrefix + "ToroidLocatorFrame", toroidBase.getBodyFixedFrame());
      toroidBeforeJointFrame.update();
      
      FrameVector toroidJointAxis = new FrameVector(toroidBeforeJointFrame, 0.0, 0.0, 1.0);
      toroidJoint = new RevoluteJoint("toroidJoint", toroidBase, toroidBeforeJointFrame, toroidJointAxis);
      toroid = ScrewTools.addRigidBody("toroid", toroidJoint, new Matrix3d(), 0.0, new Vector3d());    // TODO: set mass properties

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicReferenceFrame toroidBeforeJointFrameViz = new DynamicGraphicReferenceFrame(toroidBeforeJointFrame, registry, 0.3);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("toroidFrames", toroidBeforeJointFrameViz);
         dynamicGraphicReferenceFrames.add(toroidBeforeJointFrameViz);

         DynamicGraphicReferenceFrame toroidAfterJointFrameViz = new DynamicGraphicReferenceFrame(toroid.getParentJoint().getFrameAfterJoint(), registry, 0.1);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("toroidFrames", toroidAfterJointFrameViz);
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
      updateVisualizers();
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
      copy.getPosition(origin);
      FrameVector zAxis = new FrameVector(copy.getReferenceFrame());
      framePose.getOrientation(rotationMatrix);
      rotationMatrix.getColumn(2, zAxis.getVector());
      toroidBeforeJointFrame.set(origin, zAxis);

      copy.changeFrame(toroidBeforeJointFrame);
      copy.getOrientation(rotationMatrix);
      AxisAngle4d axisAngle = new AxisAngle4d();
      axisAngle.set(rotationMatrix);
      setQ(axisAngle.getAngle());

      updateVisualizers();
   }

   private void updateVisualizers()
   {
      for (DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame : dynamicGraphicReferenceFrames)
      {
         dynamicGraphicReferenceFrame.update();
      }
   }

   public double getQ()
   {
      return toroidQ.getDoubleValue();
   }

   private class TorusLocatorFrame extends ReferenceFrame
   {
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
      public void updateTransformToParent(Transform3D transformToParent)
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

         transformToParent.set(rotationMatrix);
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

      public void set(Transform3D toroidToWorldTransform)
      {
         toroidToWorldTransform.getRotationScale(rotationMatrix);
         rotationMatrix.getColumn(2, zAxis);
         toroidToWorldTransform.get(origin);
         update();
      }
   }
}
