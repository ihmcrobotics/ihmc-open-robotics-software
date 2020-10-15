package us.ihmc.robotics.physics;

import java.util.List;
import java.util.stream.Collectors;

import org.apache.commons.lang3.StringUtils;
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
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameTwist;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSingleContactImpulseCalculator extends SingleContactImpulseCalculator
{
   private final static boolean EXTRA_YOVARIABLES = false;

   private YoBoolean isContactClosing;
   private YoFrameVector3D collisionAxis;
   private YoFrameVector3D impulseA, impulseB;
   private YoFramePoint3D pointA, pointB;
   private YoFixedFrameSpatialVector velocityRelative;
   private YoFixedFrameSpatialVector velocitySolverInput;
   private YoMatrix collisionMatrix;
   private YoDouble collisionMatrixDet;
   private YoFixedFrameSpatialVector velocityInitialA, velocityInitialB;
   private YoFixedFrameSpatialVector velocityNoImpulseA, velocityNoImpulseB;
   private YoFixedFrameSpatialVector velocityDueToOtherImpulseA, velocityDueToOtherImpulseB;
   private YoFixedFrameSpatialVector velocityChangeA, velocityChangeB;
   private List<JointVelocityChange> jointVelocityChangeAList;
   private List<JointVelocityChange> jointVelocityChangeBList;

   public YoSingleContactImpulseCalculator(String prefix, int identifier, ReferenceFrame rootFrame, RigidBodyBasics rootBodyA,
                                           ForwardDynamicsCalculator forwardDynamicsCalculatorA, RigidBodyBasics rootBodyB,
                                           ForwardDynamicsCalculator forwardDynamicsCalculatorB, YoRegistry registry)
   {
      super(rootFrame, rootBodyA, forwardDynamicsCalculatorA, rootBodyB, forwardDynamicsCalculatorB);

      isContactClosing = new YoBoolean(prefix + "IsContactClosing" + identifier, registry);
      collisionAxis = new YoFrameVector3D(prefix + "CollisionAxis" + identifier, rootFrame, registry);
      pointA = new YoFramePoint3D(prefix + "PointA" + identifier, rootFrame, registry);
      pointB = new YoFramePoint3D(prefix + "PointB" + identifier, rootFrame, registry);

      velocityRelative = new YoFixedFrameSpatialVector(prefix + "VelocityRelative" + identifier, rootFrame, registry);

      if (EXTRA_YOVARIABLES)
      {
         velocitySolverInput = new YoFixedFrameSpatialVector(prefix + "VelocitySolverInput" + identifier, rootFrame, registry);
         collisionMatrix = new YoMatrix(prefix + "CollisionMatrix" + identifier, 4, 4, registry);
         collisionMatrixDet = new YoDouble(prefix + "CollisionMatrixDet" + identifier, registry);
      }

      impulseA = new YoFrameVector3D(prefix + "ImpulseA" + identifier, rootFrame, registry);

      if (EXTRA_YOVARIABLES)
      {
         velocityInitialA = new YoFixedFrameSpatialVector(prefix + "VelocityInitialA" + identifier, rootFrame, registry);
         velocityNoImpulseA = new YoFixedFrameSpatialVector(prefix + "VelocityNoImpulseA" + identifier, rootFrame, registry);
         velocityDueToOtherImpulseA = new YoFixedFrameSpatialVector(prefix + "VelocityDueToOtherImpulseA" + identifier, rootFrame, registry);
         velocityChangeA = new YoFixedFrameSpatialVector(prefix + "VelocityChangeA" + identifier, rootFrame, registry);
         jointVelocityChangeAList = SubtreeStreams.fromChildren(rootBodyA)
                                                  .map(joint -> toJointVelocityChange(prefix, "VelocityChangeA", identifier, joint, registry))
                                                  .collect(Collectors.toList());
      }

      if (EXTRA_YOVARIABLES)
      {
         if (rootBodyB != null)
         {
            impulseB = new YoFrameVector3D(prefix + "ImpulseB" + identifier, rootFrame, registry);
            velocityInitialB = new YoFixedFrameSpatialVector(prefix + "VelocityInitialB" + identifier, rootFrame, registry);
            velocityNoImpulseB = new YoFixedFrameSpatialVector(prefix + "VelocityNoImpulseB" + identifier, rootFrame, registry);
            velocityDueToOtherImpulseB = new YoFixedFrameSpatialVector(prefix + "VelocityDueToOtherImpulseB" + identifier, rootFrame, registry);
            velocityChangeB = new YoFixedFrameSpatialVector(prefix + "VelocityChangeB" + identifier, rootFrame, registry);
            jointVelocityChangeBList = SubtreeStreams.fromChildren(rootBodyB)
                                                     .map(joint -> toJointVelocityChange(prefix, "VelocityChangeB", identifier, joint, registry))
                                                     .collect(Collectors.toList());
         }
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

      yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicVector(impulseA.getNamePrefix(), pointA, impulseA, 2.0, impulseAppearance));

      if (impulseB != null)
      {
         yoGraphicsListRegistry.registerYoGraphic(groupName, new YoGraphicVector(impulseB.getNamePrefix(), pointB, impulseB, 2.0, impulseAppearance));
      }
   }

   public void clear()
   {
      if (isContactClosing != null)
         isContactClosing.set(false);
      if (collisionAxis != null)
         collisionAxis.setToNaN();

      if (pointA != null)
         pointA.setToNaN();
      if (pointB != null)
         pointB.setToNaN();

      if (velocityRelative != null)
         velocityRelative.setToNaN();
      if (velocitySolverInput != null)
         velocitySolverInput.setToNaN();

      if (collisionMatrix != null)
         collisionMatrix.setToNaN(3, 3);
      if (collisionMatrixDet != null)
         collisionMatrixDet.setToNaN();

      if (impulseA != null)
         impulseA.setToNaN();
      if (velocityInitialA != null)
         velocityInitialA.setToNaN();
      if (velocityNoImpulseA != null)
         velocityNoImpulseA.setToNaN();
      if (velocityDueToOtherImpulseA != null)
         velocityDueToOtherImpulseA.setToNaN();
      if (velocityChangeA != null)
         velocityChangeA.setToNaN();
      if (jointVelocityChangeAList != null)
         jointVelocityChangeAList.forEach(JointVelocityChange::setToNaN);

      if (impulseB != null)
         impulseB.setToNaN();
      if (velocityInitialB != null)
         velocityInitialB.setToNaN();
      if (velocityNoImpulseB != null)
         velocityNoImpulseB.setToNaN();
      if (velocityDueToOtherImpulseB != null)
         velocityDueToOtherImpulseB.setToNaN();
      if (velocityChangeB != null)
         velocityChangeB.setToNaN();
      if (jointVelocityChangeBList != null)
         jointVelocityChangeBList.forEach(JointVelocityChange::setToNaN);
   }

   @Override
   public void setCollision(CollisionResult collisionResult)
   {
      super.setCollision(collisionResult);

      if (collisionAxis != null)
         collisionAxis.set(collisionResult.getCollisionAxisForA());
      if (pointA != null)
         pointA.setMatchingFrame(collisionResult.getPointOnARootFrame());
      if (pointB != null)
         pointB.setMatchingFrame(collisionResult.getPointOnBRootFrame());
   }

   private final FrameVector3D mutableFrameVector = new FrameVector3D();

   @Override
   public void initialize(double dt)
   {
      super.initialize(dt);

      if (velocityInitialA != null)
      {
         getContactingBodyA().getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(getPointA(), mutableFrameVector);
         velocityInitialA.getLinearPart().setMatchingFrame(mutableFrameVector);
      }
      if (velocityNoImpulseA != null)
         velocityNoImpulseA.setMatchingFrame(getVelocityNoImpulseA());

      if (velocityInitialB != null)
      {
         getContactingBodyB().getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(getPointB(), mutableFrameVector);
         velocityInitialB.getLinearPart().setMatchingFrame(mutableFrameVector);
      }
      if (velocityNoImpulseB != null)
         velocityNoImpulseB.setMatchingFrame(getVelocityNoImpulseB());
   }

   @Override
   public void finalizeImpulse()
   {
      super.finalizeImpulse();

      if (isContactClosing != null)
         isContactClosing.set(isContactClosing());
      if (impulseA != null)
         impulseA.setMatchingFrame(getImpulseA().getLinearPart());

      if (velocityRelative != null)
         velocityRelative.setMatchingFrame(getVelocityRelative());
      if (velocitySolverInput != null)
         velocitySolverInput.setMatchingFrame(getVelocitySolverInput());
      if (collisionMatrix != null)
         collisionMatrix.set(getCollisionMatrix());
      if (collisionMatrixDet != null)
         collisionMatrixDet.set(CommonOps_DDRM.det(getCollisionMatrix()));
      if (velocityDueToOtherImpulseA != null)
         velocityDueToOtherImpulseA.setMatchingFrame(getVelocityDueToOtherImpulseA());

      if (velocityChangeA != null)
         velocityChangeA.getLinearPart().setMatchingFrame(getResponseCalculatorA().getTwistChangeProvider()
                                                                                  .getLinearVelocityOfBodyFixedPoint(getContactingBodyA(), getPointA()));

      if (jointVelocityChangeAList != null)
      {
         if (getJointVelocityChange(0) != null)
            jointVelocityChangeAList.forEach(holder -> holder.updateVelocity(getResponseCalculatorA()));
         else
            jointVelocityChangeAList.forEach(JointVelocityChange::setToZero);
      }

      if (impulseB != null)
         impulseB.setMatchingFrame(getImpulseB().getLinearPart());
      if (velocityDueToOtherImpulseB != null)
         velocityDueToOtherImpulseB.setMatchingFrame(getVelocityDueToOtherImpulseB());
      if (velocityChangeB != null)
         velocityChangeB.getLinearPart().setMatchingFrame(getResponseCalculatorB().getTwistChangeProvider()
                                                                                  .getLinearVelocityOfBodyFixedPoint(getContactingBodyB(), getPointB()));
      if (jointVelocityChangeBList != null)
      {
         if (getJointVelocityChange(1) != null)
            jointVelocityChangeBList.forEach(holder -> holder.updateVelocity(getResponseCalculatorB()));
         else
            jointVelocityChangeBList.forEach(JointVelocityChange::setToZero);
      }
   }

   private static JointVelocityChange toJointVelocityChange(String prefix, String suffix, int identifier, JointReadOnly joint, YoRegistry registry)
   {
      if (joint instanceof SixDoFJointReadOnly)
         return new SixDoFJointVelocityChange(prefix, suffix, identifier, (SixDoFJointReadOnly) joint, registry);
      else if (joint instanceof OneDoFJointReadOnly)
         return new OneDoFJointVelocityChange(prefix, suffix, identifier, (OneDoFJointReadOnly) joint, registry);
      else
         throw new IllegalStateException("Unexpected joint type: " + joint.getClass().getSimpleName());
   }

   private static interface JointVelocityChange
   {
      void setToZero();

      void setToNaN();

      void updateVelocity(MultiBodyResponseCalculator responseCalculator);

      JointReadOnly getJoint();
   }

   private static class SixDoFJointVelocityChange implements JointVelocityChange
   {
      private final SixDoFJointReadOnly joint;
      private final YoFixedFrameTwist velocityChange;

      public SixDoFJointVelocityChange(String prefix, String suffix, int identifier, SixDoFJointReadOnly joint, YoRegistry registry)
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

      public OneDoFJointVelocityChange(String prefix, String suffix, int identifier, OneDoFJointReadOnly joint, YoRegistry registry)
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
