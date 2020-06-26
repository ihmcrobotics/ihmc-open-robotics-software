package us.ihmc.robotics.physics;

import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.lang3.StringUtils;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoSingleContactImpulseCalculator extends SingleContactImpulseCalculator
{
   private final YoBoolean isContactClosing;
   private final YoFrameVector3D collisionAxis;
   private final YoFrameVector3D accumulatedSlip;
   private final YoFrameVector3D impulseA, impulseB;
   private final YoFramePoint3D pointA, pointB;
   private final YoFrameVector3D velocityRelative;
   private final YoFrameVector3D velocitySolverInput;
   private final YoMatrix collisionMatrix;
   private final YoDouble collisionMatrixDet;
   private final YoFrameVector3D velocityInitialA, velocityInitialB;
   private final YoFrameVector3D velocityNoImpulseA, velocityNoImpulseB;
   private final YoFrameVector3D velocityDueToOtherImpulseA, velocityDueToOtherImpulseB;
   private final YoFrameVector3D velocityChangeA, velocityChangeB;
   private final List<JointVelocityChange> jointVelocityChangeAList;
   private final List<JointVelocityChange> jointVelocityChangeBList;

   public YoSingleContactImpulseCalculator(String prefix, int identifier, ReferenceFrame rootFrame, RigidBodyBasics rootBodyA,
                                           ForwardDynamicsCalculator forwardDynamicsCalculatorA, RigidBodyBasics rootBodyB,
                                           ForwardDynamicsCalculator forwardDynamicsCalculatorB, YoVariableRegistry registry)
   {
      super(rootFrame, rootBodyA, forwardDynamicsCalculatorA, rootBodyB, forwardDynamicsCalculatorB);

      isContactClosing = new YoBoolean(prefix + "IsContactClosing" + identifier, registry);
      collisionAxis = new YoFrameVector3D(prefix + "CollisionAxis" + identifier, rootFrame, registry);
      accumulatedSlip = new YoFrameVector3D(prefix + "AccumulatedSlip" + identifier, rootFrame, registry);
      pointA = new YoFramePoint3D(prefix + "PointA" + identifier, rootFrame, registry);
      pointB = new YoFramePoint3D(prefix + "PointB" + identifier, rootFrame, registry);

      velocityRelative = new YoFrameVector3D(prefix + "VelocityRelative" + identifier, rootFrame, registry);
      velocitySolverInput = new YoFrameVector3D(prefix + "VelocitySolverInput" + identifier, rootFrame, registry);
      collisionMatrix = new YoMatrix(prefix + "CollisionMatrix" + identifier, 3, 3, registry);
      collisionMatrixDet = new YoDouble(prefix + "CollisionMatrixDet" + identifier, registry);

      impulseA = new YoFrameVector3D(prefix + "ImpulseA" + identifier, rootFrame, registry);
      velocityInitialA = new YoFrameVector3D(prefix + "VelocityInitialA" + identifier, rootFrame, registry);
      velocityNoImpulseA = new YoFrameVector3D(prefix + "VelocityNoImpulseA" + identifier, rootFrame, registry);
      velocityDueToOtherImpulseA = new YoFrameVector3D(prefix + "VelocityDueToOtherImpulseA" + identifier, rootFrame, registry);
      velocityChangeA = new YoFrameVector3D(prefix + "VelocityChangeA" + identifier, rootFrame, registry);
      jointVelocityChangeAList = SubtreeStreams.fromChildren(rootBodyA)
                                               .map(joint -> JointVelocityChange.toJointVelocityChange(prefix, "VelocityChangeA", identifier, joint, registry))
                                               .collect(Collectors.toList());

      if (rootBodyB != null)
      {
         impulseB = new YoFrameVector3D(prefix + "ImpulseB" + identifier, rootFrame, registry);
         velocityInitialB = new YoFrameVector3D(prefix + "VelocityInitialB" + identifier, rootFrame, registry);
         velocityNoImpulseB = new YoFrameVector3D(prefix + "VelocityNoImpulseB" + identifier, rootFrame, registry);
         velocityDueToOtherImpulseB = new YoFrameVector3D(prefix + "VelocityDueToOtherImpulseB" + identifier, rootFrame, registry);
         velocityChangeB = new YoFrameVector3D(prefix + "VelocityChangeB" + identifier, rootFrame, registry);
         jointVelocityChangeBList = SubtreeStreams.fromChildren(rootBodyB)
                                                  .map(joint -> JointVelocityChange.toJointVelocityChange(prefix,
                                                                                                          "VelocityChangeB",
                                                                                                          identifier,
                                                                                                          joint,
                                                                                                          registry))
                                                  .collect(Collectors.toList());
      }
      else
      {
         impulseB = null;
         velocityInitialB = null;
         velocityNoImpulseB = null;
         velocityDueToOtherImpulseB = null;
         velocityChangeB = null;
         jointVelocityChangeBList = null;
      }
      clear();
   }

   public void setupGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String groupName = "Physics - ContactConstraint";
      AppearanceDefinition contactAppearance = YoAppearance.Blue();
      AppearanceDefinition impulseAppearance = YoAppearance.BlackMetalMaterial();

      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicPosition(pointA.getNamePrefix(), pointA, 0.005, contactAppearance));
      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicPosition(pointB.getNamePrefix(), pointB, 0.005, contactAppearance));

      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicVector(impulseA.getNamePrefix(), pointA, impulseA, 5.0, impulseAppearance));

      if (impulseB != null)
      {
         yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicVector(impulseB.getNamePrefix(), pointB, impulseB, 2.0, impulseAppearance));
      }
   }

   public void clear()
   {
      isContactClosing.set(false);
      collisionAxis.setToNaN();
      accumulatedSlip.setToNaN();

      pointA.setToNaN();
      pointB.setToNaN();

      velocityRelative.setToNaN();
      velocitySolverInput.setToNaN();

      collisionMatrix.setToNaN(3, 3);
      collisionMatrixDet.setToNaN();

      impulseA.setToNaN();
      velocityInitialA.setToNaN();
      velocityNoImpulseA.setToNaN();
      velocityDueToOtherImpulseA.setToNaN();
      velocityChangeA.setToNaN();
      jointVelocityChangeAList.forEach(JointVelocityChange::setToNaN);

      if (impulseB != null)
      {
         impulseB.setToNaN();
         velocityInitialB.setToNaN();
         velocityNoImpulseB.setToNaN();
         velocityDueToOtherImpulseB.setToNaN();
         velocityChangeB.setToNaN();
         jointVelocityChangeBList.forEach(JointVelocityChange::setToNaN);
      }
   }

   @Override
   public void setCollision(CollisionResult collisionResult)
   {
      super.setCollision(collisionResult);

      collisionAxis.set(collisionResult.getCollisionAxisForA());
      if (collisionResult.getAccumulatedSlipForA() != null)
         accumulatedSlip.set(collisionResult.getAccumulatedSlipForA());
      else
         accumulatedSlip.setToZero();
      pointA.setMatchingFrame(collisionResult.getPointOnARootFrame());
      pointB.setMatchingFrame(collisionResult.getPointOnBRootFrame());
   }

   private final FrameVector3D mutableFrameVector = new FrameVector3D();

   @Override
   public void initialize(double dt)
   {
      super.initialize(dt);

      getContactingBodyA().getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(getPointA(), mutableFrameVector);
      velocityInitialA.setMatchingFrame(mutableFrameVector);
      velocityNoImpulseA.setMatchingFrame(getVelocityNoImpulseA());

      if (impulseB != null)
      {
         getContactingBodyB().getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(getPointB(), mutableFrameVector);
         velocityInitialB.setMatchingFrame(mutableFrameVector);
         velocityNoImpulseB.setMatchingFrame(getVelocityNoImpulseB());
      }
   }

   private final DMatrixRMaj temp = new DMatrixRMaj(3, 3);

   @Override
   public void finalizeImpulse()
   {
      super.finalizeImpulse();

      isContactClosing.set(isContactClosing());
      impulseA.setMatchingFrame(getImpulseA().getLinearPart());

      velocityRelative.setMatchingFrame(getVelocityRelative());
      velocitySolverInput.setMatchingFrame(getVelocitySolverInput());
      CommonOps_DDRM.invert(getCollisionMatrix(), temp);
      collisionMatrix.set(temp);
      collisionMatrixDet.set(CommonOps_DDRM.det(getCollisionMatrix()));
      velocityDueToOtherImpulseA.setMatchingFrame(getVelocityDueToOtherImpulseA());

      velocityChangeA.setMatchingFrame(getResponseCalculatorA().getTwistChangeProvider().getLinearVelocityOfBodyFixedPoint(getContactingBodyA(), getPointA()));

      if (getJointVelocityChange(0) != null)
         jointVelocityChangeAList.forEach(holder -> holder.updateVelocity(getResponseCalculatorA()));
      else
         jointVelocityChangeAList.forEach(JointVelocityChange::setToZero);

      if (impulseB != null)
      {
         impulseB.setMatchingFrame(getImpulseB().getLinearPart());
         velocityDueToOtherImpulseB.setMatchingFrame(getVelocityDueToOtherImpulseB());
         velocityChangeB.setMatchingFrame(getResponseCalculatorB().getTwistChangeProvider().getLinearVelocityOfBodyFixedPoint(getContactingBodyB(),
                                                                                                                              getPointB()));
         if (getJointVelocityChange(1) != null)
            jointVelocityChangeBList.forEach(holder -> holder.updateVelocity(getResponseCalculatorB()));
         else
            jointVelocityChangeBList.forEach(JointVelocityChange::setToZero);
      }
   }

   private static interface JointVelocityChange
   {
      public static JointVelocityChange toJointVelocityChange(String prefix, String suffix, int identifier, JointReadOnly joint, YoVariableRegistry registry)
      {
         if (joint instanceof SixDoFJointReadOnly)
            return new SixDoFJointVelocityChange(prefix, suffix, identifier, (SixDoFJointReadOnly) joint, registry);
         else if (joint instanceof OneDoFJointReadOnly)
            return new OneDoFJointVelocityChange(prefix, suffix, identifier, (OneDoFJointReadOnly) joint, registry);
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

      public SixDoFJointVelocityChange(String prefix, String suffix, int identifier, SixDoFJointReadOnly joint, YoVariableRegistry registry)
      {
         this.joint = joint;

         velocityChange = new YoFixedFrameTwist(prefix + StringUtils.capitalize(joint.getName()) + suffix
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

      public OneDoFJointVelocityChange(String prefix, String suffix, int identifier, OneDoFJointReadOnly joint, YoVariableRegistry registry)
      {
         this.joint = joint;

         velocityChange = new YoDouble(prefix + StringUtils.capitalize(joint.getName()) + suffix + identifier, registry);
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
