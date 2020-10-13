package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpPreserveYReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DiagnosticsWhenHangingHelper
{
   private static final boolean DEBUG = false;

   private final OneDoFJointBasics parentJoint;
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

   public DiagnosticsWhenHangingHelper(OneDoFJointBasics parentJoint, boolean preserveY, YoRegistry registry)
   {
      this(parentJoint, preserveY, false, null, registry);
   }

   public DiagnosticsWhenHangingHelper(OneDoFJointBasics parentJoint, boolean preserveY, boolean isSpineJoint,
         SideDependentList<JointBasics> topLegJointsIfSpine, YoRegistry registry)
   {
      this.parentJoint = parentJoint;
      this.isSpineJoint = isSpineJoint;
      centerOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(parentJoint, preserveY, isSpineJoint, topLegJointsIfSpine);

      belowJointCoMInZUpFrame = new YoFramePoint3D(parentJoint.getName() + "CoMInZUpFrame", centerOfMassCalculator.getReferenceFrame(), registry);
      centerOfMassPosition = new FramePoint3D(centerOfMassCalculator.getReferenceFrame());

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

   private static CenterOfMassCalculator createCenterOfMassCalculatorInJointZUpFrame(JointBasics parentJoint, boolean preserveY, boolean spineJoint,
         SideDependentList<JointBasics> topLegJointsIfSpine)
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

      MultiBodySystemReadOnly subSystem;

      if (spineJoint)
      {
         Set<JointReadOnly> jointsToConsider = new HashSet<>();
         jointsToConsider.addAll(Arrays.asList(MultiBodySystemTools.collectSupportJoints(parentJoint.getPredecessor())));
         for (JointBasics legJoint : topLegJointsIfSpine)
            legJoint.subtreeIterable().forEach(jointsToConsider::add);
         subSystem = MultiBodySystemReadOnly.toMultiBodySystemInput(new ArrayList<>(jointsToConsider));
      }
      else
      {
         subSystem = MultiBodySystemReadOnly.toMultiBodySystemInput(parentJoint.subtreeList());
      }

      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(subSystem, jointZUpFrame);

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
      centerOfMassCalculator.getReferenceFrame().update();

      centerOfMassCalculator.reset();
      centerOfMassPosition.setIncludingFrame(centerOfMassCalculator.getCenterOfMass());
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
