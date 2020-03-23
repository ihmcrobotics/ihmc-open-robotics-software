package us.ihmc.robotics.physics;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyResponseCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoSingleContactImpulseCalculator extends SingleContactImpulseCalculator
{
   private final YoFrameVector3D impulseA, impulseB;
   private final YoFramePoint3D pointA, pointB;
   private final YoFrameVector3D velocityNoImpulseA, velocityNoImpulseB;
   private final YoFrameVector3D velocityDueToOtherImpulseA, velocityDueToOtherImpulseB;
   private final List<JointVelocityChange> jointVelocityChangeAList;
   private final List<JointVelocityChange> jointVelocityChangeBList;

   public YoSingleContactImpulseCalculator(int identifier, ReferenceFrame rootFrame, RigidBodyBasics rootBodyA,
                                           ForwardDynamicsCalculator forwardDynamicsCalculatorA, RigidBodyBasics rootBodyB,
                                           ForwardDynamicsCalculator forwardDynamicsCalculatorB, YoVariableRegistry registry)
   {
      super(rootFrame, rootBodyA, forwardDynamicsCalculatorA, rootBodyB, forwardDynamicsCalculatorB);

      pointA = new YoFramePoint3D("pointA" + identifier, rootFrame, registry);
      pointB = new YoFramePoint3D("pointB" + identifier, rootFrame, registry);

      impulseA = new YoFrameVector3D("impulseA" + identifier, rootFrame, registry);
      velocityNoImpulseA = new YoFrameVector3D("velocityNoImpulseA" + identifier, rootFrame, registry);
      velocityDueToOtherImpulseA = new YoFrameVector3D("velocityDueToOtherImpulseA" + identifier, rootFrame, registry);
      jointVelocityChangeAList = SubtreeStreams.fromChildren(rootBodyA).map(joint -> JointVelocityChange.toJointVelocityChange(identifier, joint, registry))
                                               .collect(Collectors.toList());

      if (rootBodyB != null)
      {
         impulseB = new YoFrameVector3D("impulseB" + identifier, rootFrame, registry);
         velocityNoImpulseB = new YoFrameVector3D("velocityNoImpulseB" + identifier, rootFrame, registry);
         velocityDueToOtherImpulseB = new YoFrameVector3D("velocityDueToOtherImpulseB" + identifier, rootFrame, registry);
         jointVelocityChangeBList = SubtreeStreams.fromChildren(rootBodyB).map(joint -> JointVelocityChange.toJointVelocityChange(identifier, joint, registry))
                                                  .collect(Collectors.toList());
      }
      else
      {
         impulseB = null;
         velocityNoImpulseB = null;
         velocityDueToOtherImpulseB = null;
         jointVelocityChangeBList = null;
      }
   }

   public void setupGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String groupName = "Physics - ContactConstraint";
      AppearanceDefinition contactAppearance = YoAppearance.Blue();
      AppearanceDefinition impulseAppearance = YoAppearance.BlackMetalMaterial();

      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicPosition(pointA.getNamePrefix(), pointA, 0.005, contactAppearance));
      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicPosition(pointB.getNamePrefix(), pointB, 0.005, contactAppearance));

      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicVector(impulseA.getNamePrefix(), pointA, impulseA, impulseAppearance));

      if (impulseB != null)
      {
         yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicVector(impulseB.getNamePrefix(), pointB, impulseB, impulseAppearance));
      }
   }

   public void clear()
   {
      pointA.setToNaN();
      pointB.setToNaN();

      impulseA.setToNaN();
      velocityNoImpulseA.setToNaN();
      velocityDueToOtherImpulseA.setToNaN();
      jointVelocityChangeAList.forEach(JointVelocityChange::setToNaN);

      if (impulseB != null)
      {
         impulseB.setToNaN();
         velocityNoImpulseB.setToNaN();
         velocityDueToOtherImpulseB.setToNaN();
         jointVelocityChangeBList.forEach(JointVelocityChange::setToNaN);
      }
   }

   @Override
   public void setCollision(CollisionResult collisionResult)
   {
      super.setCollision(collisionResult);

      pointA.setMatchingFrame(getPointA());
      pointB.setMatchingFrame(getPointB());
   }

   @Override
   public void initialize(double dt)
   {
      super.initialize(dt);

      velocityNoImpulseA.setMatchingFrame(getVelocityNoImpulseA());

      if (impulseB != null)
      {
         velocityNoImpulseB.setMatchingFrame(getVelocityNoImpulseB());
      }
   }

   @Override
   public void finalizeImpulse()
   {
      super.finalizeImpulse();

      impulseA.setMatchingFrame(getImpulseA().getLinearPart());
      velocityDueToOtherImpulseA.setMatchingFrame(getVelocityDueToOtherImpulseA());
      if (getJointVelocityChange(0) != null)
         jointVelocityChangeAList.forEach(holder -> holder.updateVelocity(getResponseCalculatorA()));
      else
         jointVelocityChangeAList.forEach(JointVelocityChange::setToZero);

      if (impulseB != null)
      {
         impulseB.setMatchingFrame(getImpulseB().getLinearPart());
         velocityDueToOtherImpulseB.setMatchingFrame(getVelocityDueToOtherImpulseB());
         if (getJointVelocityChange(1) != null)
            jointVelocityChangeBList.forEach(holder -> holder.updateVelocity(getResponseCalculatorB()));
         else
            jointVelocityChangeBList.forEach(JointVelocityChange::setToZero);
      }
   }

   private static interface JointVelocityChange
   {
      public static JointVelocityChange toJointVelocityChange(int identifier, JointReadOnly joint, YoVariableRegistry registry)
      {
         if (joint instanceof SixDoFJointReadOnly)
            return new SixDoFJointVelocityChange(identifier, (SixDoFJointReadOnly) joint, registry);
         else if (joint instanceof OneDoFJointReadOnly)
            return new OneDoFJointVelocityChange(identifier, (OneDoFJointReadOnly) joint, registry);
         else
            throw new IllegalStateException("Unexpected joint type: " + joint.getClass().getSimpleName());
      }

      void setToZero();

      void setToNaN();

      void updateVelocity(MultiBodyResponseCalculator responseCalculator);

      JointReadOnly getJoint();
   }

   private static class SixDoFJointVelocityChange implements JointVelocityChange
   {
      private final SixDoFJointReadOnly joint;
      private final YoFixedFrameTwist velocityChange;

      public SixDoFJointVelocityChange(int identifier, SixDoFJointReadOnly joint, YoVariableRegistry registry)
      {
         this.joint = joint;

         velocityChange = new YoFixedFrameTwist(joint.getName() + "VelocityChangeA"
               + identifier, joint.getFrameAfterJoint(), joint.getFrameBeforeJoint(), joint.getFrameAfterJoint(), registry);
      }

      @Override
      public void setToZero()
      {
         velocityChange.setToZero();
      }

      @Override
      public void setToNaN()
      {
         velocityChange.setToNaN();
      }

      @Override
      public void updateVelocity(MultiBodyResponseCalculator responseCalculator)
      {
         velocityChange.set(responseCalculator.getJointTwistChange(joint));
      }

      @Override
      public SixDoFJointReadOnly getJoint()
      {
         return joint;
      }
   }

   private static class OneDoFJointVelocityChange implements JointVelocityChange
   {
      private final OneDoFJointReadOnly joint;
      private final YoDouble velocityChange;

      public OneDoFJointVelocityChange(int identifier, OneDoFJointReadOnly joint, YoVariableRegistry registry)
      {
         this.joint = joint;

         velocityChange = new YoDouble(joint.getName() + "VelocityChangeA" + identifier, registry);
      }

      @Override
      public void setToZero()
      {
         velocityChange.set(0.0);
      }

      @Override
      public void setToNaN()
      {
         velocityChange.setToNaN();
      }

      @Override
      public void updateVelocity(MultiBodyResponseCalculator responseCalculator)
      {
         velocityChange.set(responseCalculator.getJointTwistChange(joint));
      }

      @Override
      public OneDoFJointReadOnly getJoint()
      {
         return joint;
      }
   }
}
