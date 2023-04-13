package us.ihmc.avatar.multiContact;

import java.util.HashSet;
import java.util.Set;

import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.SelectionMatrix3DMessage;
import us.ihmc.avatar.multiContact.RobotTransformOptimizer.RigidBodyPairAngularErrorCalculator;
import us.ihmc.avatar.multiContact.RobotTransformOptimizer.RigidBodyPairLinearErrorCalculator;
import us.ihmc.avatar.multiContact.RobotTransformOptimizer.RigidBodyPairSpatialErrorCalculator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class MultiContactScriptMatcher
{
   private final FullHumanoidRobotModel scriptFullRobotModel;
   private final OneDoFJointBasics[] allScriptJoints;
   private final int jointNameHash;
   private final ReferenceFrameHashCodeResolver frameResolver;
   private final RobotTransformOptimizer optimizer;

   private final Set<RigidBodyBasics> defaultScriptBodiesToMatch = new HashSet<>();

   public MultiContactScriptMatcher(FullHumanoidRobotModelFactory factory, FullHumanoidRobotModel controllerFullRobotModel)
   {
      scriptFullRobotModel = factory.createFullRobotModel();
      allScriptJoints = FullRobotModelUtils.getAllJointsExcludingHands(scriptFullRobotModel);
      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(allScriptJoints,
                                                                           scriptFullRobotModel.getForceSensorDefinitions(),
                                                                           scriptFullRobotModel.getIMUDefinitions());
      frameResolver = new ReferenceFrameHashCodeResolver();
      frameResolver.putAllFullRobotModelReferenceFrames(scriptFullRobotModel);
      optimizer = new RobotTransformOptimizer(controllerFullRobotModel.getElevator(), scriptFullRobotModel.getElevator());

      defaultScriptBodiesToMatch.add(scriptFullRobotModel.getPelvis());
      defaultScriptBodiesToMatch.add(scriptFullRobotModel.getChest());
      for (RobotSide robotSide : RobotSide.values)
      {
         defaultScriptBodiesToMatch.add(scriptFullRobotModel.getHand(robotSide));
         defaultScriptBodiesToMatch.add(scriptFullRobotModel.getFoot(robotSide));
      }
   }

   public void computeTransform(KinematicsToolboxSnapshotDescription description)
   {
      updateScriptRobotConfiguration(description.getControllerConfiguration());

      optimizer.clearErrorCalculators();
      optimizer.addDefaultRigidBodyLinearErrorCalculators((controllerBody, scriptBody) -> defaultScriptBodiesToMatch.contains(scriptBody));

      for (SixDoFMotionControlAnchorDescription anchorDescription : description.getSixDoFAnchors())
      {
         if (anchorDescription.isContactState() || anchorDescription.isTrackingController())
         {
            KinematicsToolboxRigidBodyMessage inputMessage = anchorDescription.getInputMessage();
            SelectionMatrix3DMessage linearSelection = inputMessage.getLinearSelectionMatrix();
            SelectionMatrix3DMessage angularSelection = inputMessage.getAngularSelectionMatrix();
            RigidBodyTransform controlFramePose = new RigidBodyTransform(inputMessage.getControlFrameOrientationInEndEffector(),
                                                                         inputMessage.getControlFramePositionInEndEffector());

            double weight = 1.0;
            if (anchorDescription.isContactState())
               weight *= 10.0;
            if (anchorDescription.isTrackingController())
               weight *= 10.0;

            if (isAllFalse(linearSelection))
            {
               if (isAllFalse(angularSelection))
                  continue;

               RigidBodyPairAngularErrorCalculator errorCalculator = optimizer.addAngularRigidBodyErrorCalculator(anchorDescription.getRigidBodyName());
               errorCalculator.setControlFrameOffset(controlFramePose);
               packSelectionMatrix3D(angularSelection, errorCalculator.getSelectionMatrix());
               setWeightMatrix3D(weight, errorCalculator.getWeightMatrix());
            }
            else if (isAllFalse(angularSelection))
            {
               RigidBodyPairLinearErrorCalculator errorCalculator = optimizer.addLinearRigidBodyErrorCalculator(anchorDescription.getRigidBodyName());
               errorCalculator.setControlFrameOffset(controlFramePose);
               packSelectionMatrix3D(linearSelection, errorCalculator.getSelectionMatrix());
               setWeightMatrix3D(weight, errorCalculator.getWeightMatrix());
            }
            else
            {
               RigidBodyPairSpatialErrorCalculator errorCalculator = optimizer.addSpatialRigidBodyErrorCalculator(anchorDescription.getRigidBodyName());
               errorCalculator.setControlFrameOffset(controlFramePose);
               packSelectionMatrix3D(linearSelection, errorCalculator.getSelectionMatrix().getLinearPart());
               packSelectionMatrix3D(angularSelection, errorCalculator.getSelectionMatrix().getAngularPart());
               setWeightMatrix3D(weight, errorCalculator.getWeightMatrix().getLinearPart());
               setWeightMatrix3D(weight, errorCalculator.getWeightMatrix().getAngularPart());
            }
         }
      }

      optimizer.compute();
   }

   public RigidBodyTransform getScriptTransform()
   {
      return optimizer.getTransformFromBToA();
   }

   private void updateScriptRobotConfiguration(RobotConfigurationData configuration)
   {
//      if (configuration.getJointNameHash() != jointNameHash)
//         throw new RuntimeException("Hashes are different.");
      scriptFullRobotModel.getRootJoint().getJointPose().set(configuration.getRootPosition(), configuration.getRootOrientation());
      for (int i = 0; i < configuration.getJointAngles().size(); i++)
         allScriptJoints[i].setQ(configuration.getJointAngles().get(i));
      scriptFullRobotModel.updateFrames();
   }

   private void packSelectionMatrix3D(SelectionMatrix3DMessage message, SelectionMatrix3D selectionMatrixToPack)
   {
      selectionMatrixToPack.setSelectionFrame(frameResolver.getReferenceFrame(message.getSelectionFrameId()));
      selectionMatrixToPack.selectXAxis(message.getXSelected());
      selectionMatrixToPack.selectYAxis(message.getYSelected());
      selectionMatrixToPack.selectZAxis(message.getZSelected());
   }

   private static boolean isAllFalse(SelectionMatrix3DMessage selection)
   {
      return !selection.getXSelected() && !selection.getYSelected() && !selection.getZSelected();
   }

   private static void setWeightMatrix3D(double weight, WeightMatrix3D weightMatrixToPack)
   {
      weightMatrixToPack.setWeights(weight, weight, weight);
   }
}
