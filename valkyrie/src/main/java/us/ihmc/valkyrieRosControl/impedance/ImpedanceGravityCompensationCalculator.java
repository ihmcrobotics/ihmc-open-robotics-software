package us.ihmc.valkyrieRosControl.impedance;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.MultiContactForceOptimizer;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.MultiContactSupportRegionSolverInput;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ImpedanceGravityCompensationCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble gravityCompensationGain = new YoDouble("gravityCompensationGain", registry);

   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final List<OneDoFJointBasics> jointsToAddGravityCompensation = new ArrayList<>();
   private final Map<OneDoFJointBasics, YoDouble> gravityCompensationTorques = new HashMap<>();

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final DMatrixRMaj desiredAccelerations;
   private final double robotMass;

   private final MultiContactSupportRegionSolverInput solverInput = new MultiContactSupportRegionSolverInput();
   private final MultiContactForceOptimizer forceOptimizer = new MultiContactForceOptimizer();
   private final CenterOfMassReferenceFrame centerOfMassReferenceFrame;

   private final Map<RigidBodyBasics, TIntArrayList> bodyToIndex = new HashMap<>();

   private final FramePose3D contactPointPoseAlignedWithWorld = new FramePose3D();
   private final FramePoint3D centerOfMass = new FramePoint3D();
   private final PoseReferenceFrame contactPointFrame = new PoseReferenceFrame("contactPointFrame", ReferenceFrame.getWorldFrame());

   private final Wrench totalWrench = new Wrench();
   private final Vector3D resolvedForceInWorld = new Vector3D();
   private final Wrench tmpWrench = new Wrench();

   public ImpedanceGravityCompensationCalculator(OneDoFJointBasics[] controlledOneDoFJoints, double robotMass, YoRegistry parentRegistry)
   {
      this.controlledOneDoFJoints = controlledOneDoFJoints;
      MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledOneDoFJoints);
      inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
      inverseDynamicsCalculator.setGravitionalAcceleration(-9.81);

      desiredAccelerations = new DMatrixRMaj(controlledOneDoFJoints.length, 1);
      centerOfMassReferenceFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), multiBodySystemInput.getRootBody());
      this.robotMass = robotMass;

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         gravityCompensationTorques.put(controlledOneDoFJoints[i], new YoDouble("tau_gravComp_" + controlledOneDoFJoints[i].getName(), registry));
      }

      parentRegistry.addChild(registry);
   }

   public void registerGravityCompensationJoint(OneDoFJointBasics joint)
   {
      jointsToAddGravityCompensation.add(joint);
   }

   public void compute(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      gravityCompensationGain.set(EuclidCoreTools.clamp(gravityCompensationGain.getValue(), 0.0, 1.0));

      centerOfMassReferenceFrame.update();

      centerOfMass.setToZero(centerOfMassReferenceFrame);
      centerOfMass.changeFrame(ReferenceFrame.getWorldFrame());
      forceOptimizer.solve(solverInput, centerOfMass.getX(), centerOfMass.getY());

      for (RigidBodyBasics contactingBody : bodyToIndex.keySet())
      {
         totalWrench.setToZero(contactingBody.getBodyFixedFrame(), contactingBody.getBodyFixedFrame());

         TIntArrayList indices = bodyToIndex.get(contactingBody);
         for (int i = 0; i < indices.size(); i++)
         {
            int contactPointIndex = indices.get(i);

            contactPointPoseAlignedWithWorld.getPosition().set(solverInput.getContactPointPositions().get(contactPointIndex));
            contactPointFrame.setPoseAndUpdate(contactPointPoseAlignedWithWorld);
            tmpWrench.setToZero(contactingBody.getBodyFixedFrame(), contactPointFrame);

            resolvedForceInWorld.set(forceOptimizer.getResolvedForce(contactPointIndex));
            resolvedForceInWorld.scale(9.81 * robotMass);

            tmpWrench.getLinearPart().set(resolvedForceInWorld);
            tmpWrench.changeFrame(contactingBody.getBodyFixedFrame());
            totalWrench.add(tmpWrench);
         }

         inverseDynamicsCalculator.setExternalWrench(contactingBody, totalWrench);
      }

      inverseDynamicsCalculator.compute(desiredAccelerations);

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         gravityCompensationTorques.get(controlledOneDoFJoints[i]).set(0.0);
      }

      for (int i = 0; i < jointsToAddGravityCompensation.size(); i++)
      {
         OneDoFJointBasics joint = jointsToAddGravityCompensation.get(i);
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         double desiredFeedbackTorque = jointDesiredOutput.getDesiredTorque();
         double desiredFeedForwardTorque = gravityCompensationGain.getDoubleValue() * inverseDynamicsCalculator.getComputedJointTau(joint).get(0, 0);
         jointDesiredOutput.setDesiredTorque(desiredFeedbackTorque + desiredFeedForwardTorque);

         gravityCompensationTorques.get(joint).set(desiredFeedForwardTorque);
      }
   }
}
