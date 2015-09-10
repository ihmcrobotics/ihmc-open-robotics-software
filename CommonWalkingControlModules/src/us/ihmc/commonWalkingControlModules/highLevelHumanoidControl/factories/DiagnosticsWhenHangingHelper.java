package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpPreserveYReferenceFrame;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class DiagnosticsWhenHangingHelper
{
   private static final boolean DEBUG = false;

   private final OneDoFJoint parentJoint;
   private final boolean isSpineJoint;
   
   private final CenterOfMassCalculator centerOfMassCalculator;
   private final YoFramePoint belowJointCoMInZUpFrame;
   private final YoFrameVector yoJointAxis, yoJointToCenterOfMass, yoForceVector;
   
   private final FramePoint centerOfMassPosition;
   private final FrameVector jointAxis = new FrameVector();
   private final FrameVector jointToCenterOfMass = new FrameVector();
   private FrameVector forceVector = new FrameVector();
   private FrameVector rCrossFVector = new FrameVector();
   
   private final DoubleYoVariable totalMass;
   private final DoubleYoVariable estimatedTorque, torqueOffset, appliedTorque;
   
   private final DoubleYoVariable torqueCorrectionAlpha;

   public DiagnosticsWhenHangingHelper(OneDoFJoint parentJoint, boolean preserveY, YoVariableRegistry registry)
   {
      this(parentJoint, preserveY, false, null, registry);
   }
   
   public DiagnosticsWhenHangingHelper(OneDoFJoint parentJoint, boolean preserveY, boolean isSpineJoint, SideDependentList<InverseDynamicsJoint> topLegJointsIfSpine, YoVariableRegistry registry)
   {
      this.parentJoint = parentJoint;
      this.isSpineJoint = isSpineJoint;
      centerOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(parentJoint, preserveY, isSpineJoint, topLegJointsIfSpine);

      belowJointCoMInZUpFrame = new YoFramePoint(parentJoint.getName() + "CoMInZUpFrame", centerOfMassCalculator.getDesiredFrame(), registry);
      centerOfMassPosition = new FramePoint(centerOfMassCalculator.getDesiredFrame());
      
      yoJointAxis = new YoFrameVector(parentJoint.getName() + "JointAxis", ReferenceFrame.getWorldFrame(), registry);
      yoJointToCenterOfMass = new YoFrameVector(parentJoint.getName() + "JointToCoM", ReferenceFrame.getWorldFrame(), registry);
      yoForceVector = new YoFrameVector(parentJoint.getName() + "ForceVector", ReferenceFrame.getWorldFrame(), registry);
      
      estimatedTorque = new DoubleYoVariable("tau_est_" + parentJoint.getName(), registry);
      torqueOffset = new DoubleYoVariable("tau_off_" + parentJoint.getName(), registry);
      appliedTorque = new DoubleYoVariable("tau_app_" + parentJoint.getName(), registry);
      
      totalMass = new DoubleYoVariable("totalMass_" + parentJoint.getName(), registry);
      
      torqueCorrectionAlpha = new DoubleYoVariable("torqueCorrectionAlpha_" + parentJoint.getName(), registry);
      torqueCorrectionAlpha.set(0.001);
   }

   private static CenterOfMassCalculator createCenterOfMassCalculatorInJointZUpFrame(InverseDynamicsJoint parentJoint, boolean preserveY, boolean spineJoint, SideDependentList<InverseDynamicsJoint> topLegJointsIfSpine)
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

   public double getTorqueToApply(double feedbackCorrectionTorque, boolean adaptTorqueOffset)
   {
      appliedTorque.set(feedbackCorrectionTorque + estimatedTorque.getDoubleValue()); 
      
      if (adaptTorqueOffset) // && (Math.abs(appliedTorque.getDoubleValue()) < 10.0))
      {
         torqueOffset.sub(feedbackCorrectionTorque * torqueCorrectionAlpha.getDoubleValue());
         
         if (torqueOffset.getDoubleValue() > 15.0) torqueOffset.set(15.0);
         if (torqueOffset.getDoubleValue() < -15.0) torqueOffset.set(-15.0);
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
   
   public DoubleYoVariable getEstimatedTorqueYoVariable()
   {
      return estimatedTorque;
   }
   
   public DoubleYoVariable getAppliedTorqueYoVariable()
   {
      return appliedTorque;
   }
   
   private final FrameVector jointAxisInWorld = new FrameVector();
   private final FrameVector jointToCenterOfMassInWorld = new FrameVector(jointToCenterOfMass);
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
      
      FrameVector forceVectorInWorld = new FrameVector(forceVector);
      forceVectorInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      yoForceVector.set(forceVectorInWorld);
      
      rCrossFVector.setToZero(jointAxis.getReferenceFrame());
      rCrossFVector.cross(forceVector, jointToCenterOfMass);
      
      estimatedTorque.set(rCrossFVector.dot(jointAxis));
      if (isSpineJoint) estimatedTorque.mul(-1.0);
   }
   
   public void addOffsetToEstimatedTorque()
   {
      estimatedTorque.add(torqueOffset);
   }

   public DoubleYoVariable getTorqueOffsetVariable()
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
