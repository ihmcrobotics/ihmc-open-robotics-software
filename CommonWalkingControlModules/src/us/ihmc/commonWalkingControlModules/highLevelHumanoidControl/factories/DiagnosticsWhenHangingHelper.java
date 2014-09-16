package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

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
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class DiagnosticsWhenHangingHelper
{
   private static final boolean DEBUG = true;

   private final OneDoFJoint parentJoint;
   
   private final CenterOfMassCalculator centerOfMassCalculator;
   private final YoFramePoint belowJointCoMInZUpFrame;
   private final YoFrameVector yoJointAxis, yoJointToCenterOfMass, yoForceVector;
   
   private final FramePoint centerOfMassPosition;
   private final FrameVector jointAxis = new FrameVector();
   private final FrameVector jointToCenterOfMass = new FrameVector();
   private FrameVector forceVector = new FrameVector();
   private FrameVector rCrossFVector = new FrameVector();
   
   private final DoubleYoVariable totalMass;
   private final DoubleYoVariable estimatedTorque;

   public DiagnosticsWhenHangingHelper(OneDoFJoint parentJoint, boolean preserveY, YoVariableRegistry registry)
   {
      this.parentJoint = parentJoint;
      centerOfMassCalculator = createCenterOfMassCalculatorInJointZUpFrame(parentJoint, preserveY);

      belowJointCoMInZUpFrame = new YoFramePoint(parentJoint.getName() + "CoMInZUpFrame", centerOfMassCalculator.getDesiredFrame(), registry);
      centerOfMassPosition = new FramePoint(centerOfMassCalculator.getDesiredFrame());
      
      yoJointAxis = new YoFrameVector(parentJoint.getName() + "JointAxis", ReferenceFrame.getWorldFrame(), registry);
      yoJointToCenterOfMass = new YoFrameVector(parentJoint.getName() + "JointToCoM", ReferenceFrame.getWorldFrame(), registry);
      yoForceVector = new YoFrameVector(parentJoint.getName() + "ForceVector", ReferenceFrame.getWorldFrame(), registry);
      
      estimatedTorque = new DoubleYoVariable("tau_est_" + parentJoint.getName(), registry);
      totalMass = new DoubleYoVariable("totalMass_" + parentJoint.getName(), registry);
   }

   private static CenterOfMassCalculator createCenterOfMassCalculatorInJointZUpFrame(InverseDynamicsJoint parentJoint, boolean preserveY)
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

      RigidBody[] rigidBodies = ScrewTools.computeRigidBodiesAfterThisJoint(parentJoint);
      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rigidBodies, jointZUpFrame);

      return centerOfMassCalculator;
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
   }


}
