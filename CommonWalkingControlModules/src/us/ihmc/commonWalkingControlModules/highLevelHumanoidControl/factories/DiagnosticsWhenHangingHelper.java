package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ZUpFrame;
import us.ihmc.utilities.math.geometry.ZUpPreserveYReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class DiagnosticsWhenHangingHelper
{
   private static final boolean DEBUG = true;

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
   
   private final PDController torqueCorrectionPdController;

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
      
      torqueCorrectionPdController = new PDController("tau_corr_" + parentJoint.getName(), registry);
      torqueCorrectionPdController.setProportionalGain(0.1);
      torqueCorrectionPdController.setDerivativeGain(0.02);
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

   public double getTorqueToApply(double desiredTorque)
   {
      if (appliedTorque.getDoubleValue() < 4)
           torqueOffset.set(torqueCorrectionPdController.compute(torqueOffset.getDoubleValue(), estimatedTorque.getDoubleValue() - desiredTorque, 1.0, 1.0));
      
      appliedTorque.set(desiredTorque + estimatedTorque.getDoubleValue() - torqueOffset.getDoubleValue());
      return appliedTorque.getDoubleValue();
   }
   
   public double getEstimatedTorque()
   {
      return estimatedTorque.getDoubleValue();
   }
   
   public void update()
   {
      centerOfMassCalculator.getDesiredFrame().update();

      centerOfMassCalculator.compute();
      centerOfMassCalculator.getCenterOfMass(centerOfMassPosition);
      belowJointCoMInZUpFrame.set(centerOfMassPosition);
      
      jointAxis.setIncludingFrame(parentJoint.getJointAxis());
      jointAxis.changeFrame(parentJoint.getFrameAfterJoint());
      
      FrameVector jointAxisInWorld = new FrameVector(jointAxis);
      jointAxisInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      yoJointAxis.set(jointAxisInWorld);
      
      centerOfMassPosition.changeFrame(jointAxis.getReferenceFrame());
      jointToCenterOfMass.setIncludingFrame(centerOfMassPosition);
      
      FrameVector jointToCenterOfMassInWorld = new FrameVector(jointToCenterOfMass);
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


}
