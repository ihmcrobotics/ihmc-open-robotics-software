package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpPreserveYReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;

public class DiagnosticsWhenHangingHelper
{
   private static final boolean DEBUG = false;

   private final OneDoFJoint parentJoint;
   private final boolean isSpineJoint;

   private final CenterOfMassCalculator centerOfMassCalculator;
   private final YoFramePoint3D belowJointCoMInZUpFrame;
   private final YoFrameVector3D yoJointAxis, yoJointToCenterOfMass, yoForceVector;

   private final FramePoint3D centerOfMassPosition;
   private final FrameVector3D jointAxis = new FrameVector3D();
   private final FrameVector3D jointToCenterOfMass = new FrameVector3D();
   private FrameVector3D forceVector = new FrameVector3D();
   private FrameVector3D rCrossFVector = new FrameVector3D();

   private final YoDouble totalMass;
   private final YoDouble estimatedTorque, torqueOffset, appliedTorque;

   private final YoDouble torqueCorrectionAlpha;

   public DiagnosticsWhenHangingHelper(OneDoFJoint parentJoint, boolean preserveY, YoVariableRegistry registry)
   {
      this(parentJoint, preserveY, false, null, registry);
   }

   public DiagnosticsWhenHangingHelper(OneDoFJoint parentJoint, boolean preserveY, boolean isSpineJoint,
         SideDependentList<InverseDynamicsJoint> topLegJointsIfSpine, YoVariableRegistry registry)
   {
      this.parentJoint = parentJoint;
      this.isSpineJoint = isSpineJoint;
      centerOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(parentJoint, preserveY, isSpineJoint, topLegJointsIfSpine);

      belowJointCoMInZUpFrame = new YoFramePoint3D(parentJoint.getName() + "CoMInZUpFrame", centerOfMassCalculator.getDesiredFrame(), registry);
      centerOfMassPosition = new FramePoint3D(centerOfMassCalculator.getDesiredFrame());

      yoJointAxis = new YoFrameVector3D(parentJoint.getName() + "JointAxis", ReferenceFrame.getWorldFrame(), registry);
      yoJointToCenterOfMass = new YoFrameVector3D(parentJoint.getName() + "JointToCoM", ReferenceFrame.getWorldFrame(), registry);
      yoForceVector = new YoFrameVector3D(parentJoint.getName() + "ForceVector", ReferenceFrame.getWorldFrame(), registry);

      estimatedTorque = new YoDouble("tau_est_" + parentJoint.getName(), registry);
      torqueOffset = new YoDouble("tau_off_" + parentJoint.getName(), registry);
      appliedTorque = new YoDouble("tau_app_" + parentJoint.getName(), registry);

      totalMass = new YoDouble("totalMass_" + parentJoint.getName(), registry);

      torqueCorrectionAlpha = new YoDouble("torqueCorrectionAlpha_" + parentJoint.getName(), registry);
      torqueCorrectionAlpha.set(0.001);
   }

   private static CenterOfMassCalculator createCenterOfMassCalculatorInJointZUpFrame(InverseDynamicsJoint parentJoint, boolean preserveY, boolean spineJoint,
         SideDependentList<InverseDynamicsJoint> topLegJointsIfSpine)
   {
      if (DEBUG)
         System.out.println("parentJoint = " + parentJoint);

      ReferenceFrame jointFrame = parentJoint.getFrameAfterJoint();
      if (DEBUG)
         System.out.println("jointFrame = " + jointFrame);

      String jointName = parentJoint.getName();
      if (DEBUG)
         System.out.println("jointName = " + jointName);

      ReferenceFrame jointZUpFrame;

      if (preserveY)
      {
         jointZUpFrame = new ZUpPreserveYReferenceFrame(ReferenceFrame.getWorldFrame(), jointFrame, jointName + "ZUp");
      }
      else
      {
         jointZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), jointFrame, jointName + "ZUp");
      }

      ArrayList<RigidBody> rigidBodies = new ArrayList<RigidBody>();

      if (spineJoint)
      {
         ScrewTools.computeRigidBodiesFromRootToThisJoint(rigidBodies, parentJoint);
         for (InverseDynamicsJoint legJoint : topLegJointsIfSpine)
         {
            ScrewTools.computeRigidBodiesAfterThisJoint(rigidBodies, legJoint);
         }
      }
      else
      {
         ScrewTools.computeRigidBodiesAfterThisJoint(rigidBodies, parentJoint);
      }

      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rigidBodies, jointZUpFrame);

      return centerOfMassCalculator;
   }

   public double getTorqueToApply(double feedbackCorrectionTorque, boolean adaptTorqueOffset, double maxTorqueOffset)
   {
      appliedTorque.set(feedbackCorrectionTorque + estimatedTorque.getDoubleValue());

      if (adaptTorqueOffset) // && (Math.abs(appliedTorque.getDoubleValue()) < 10.0))
      {
         torqueOffset.sub(feedbackCorrectionTorque * torqueCorrectionAlpha.getDoubleValue());

         if (torqueOffset.getDoubleValue() > maxTorqueOffset)
            torqueOffset.set(maxTorqueOffset);
         if (torqueOffset.getDoubleValue() < -maxTorqueOffset)
            torqueOffset.set(-maxTorqueOffset);
      }

      return appliedTorque.getDoubleValue() - torqueOffset.getDoubleValue();
   }

   public double getEstimatedTorque()
   {
      return estimatedTorque.getDoubleValue();
   }

   public double getAppliedTorque()
   {
      return appliedTorque.getDoubleValue();
   }

   public YoDouble getEstimatedTorqueYoVariable()
   {
      return estimatedTorque;
   }

   public YoDouble getAppliedTorqueYoVariable()
   {
      return appliedTorque;
   }

   private final FrameVector3D jointAxisInWorld = new FrameVector3D();
   private final FrameVector3D jointToCenterOfMassInWorld = new FrameVector3D(jointToCenterOfMass);

   public void update()
   {
      centerOfMassCalculator.getDesiredFrame().update();

      centerOfMassCalculator.compute();
      centerOfMassCalculator.getCenterOfMass(centerOfMassPosition);
      belowJointCoMInZUpFrame.set(centerOfMassPosition);

      jointAxis.setIncludingFrame(parentJoint.getJointAxis());
      jointAxis.changeFrame(parentJoint.getFrameAfterJoint());

      jointAxisInWorld.setIncludingFrame(jointAxis);
      jointAxisInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      yoJointAxis.set(jointAxisInWorld);

      centerOfMassPosition.changeFrame(jointAxis.getReferenceFrame());
      jointToCenterOfMass.setIncludingFrame(centerOfMassPosition);

      jointToCenterOfMassInWorld.setIncludingFrame(jointToCenterOfMass);
      jointToCenterOfMassInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      yoJointToCenterOfMass.set(jointToCenterOfMassInWorld);

      totalMass.set(centerOfMassCalculator.getTotalMass());

      forceVector.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -9.81 * totalMass.getDoubleValue());
      forceVector.changeFrame(jointAxis.getReferenceFrame());

      FrameVector3D forceVectorInWorld = new FrameVector3D(forceVector);
      forceVectorInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      yoForceVector.set(forceVectorInWorld);

      rCrossFVector.setToZero(jointAxis.getReferenceFrame());
      rCrossFVector.cross(forceVector, jointToCenterOfMass);

      estimatedTorque.set(rCrossFVector.dot(jointAxis));
      if (isSpineJoint)
         estimatedTorque.mul(-1.0);
   }

   public void addOffsetToEstimatedTorque()
   {
      estimatedTorque.add(torqueOffset);
   }

   public YoDouble getTorqueOffsetVariable()
   {
      return torqueOffset;
   }

   public double getTorqueOffset()
   {
      return torqueOffset.getDoubleValue();
   }

   public void setTorqueOffset(double torqueOffset)
   {
      this.torqueOffset.set(torqueOffset);
   }

   public void setAppliedTorque(double appliedTorque)
   {
      this.appliedTorque.set(appliedTorque);
   }

}
