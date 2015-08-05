package us.ihmc.valkyrie.logProcessor;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorFunction;
import us.ihmc.darpaRoboticsChallenge.logProcessor.LogDataProcessorHelper;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class DiagnosticLogProcessorFunction implements LogDataProcessorFunction
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArrayList<OneDoFJoint> oneDoFJoints = new ArrayList<OneDoFJoint>();
   private final LinkedHashMap<OneDoFJoint, DiagnosticsWhenHangingHelper> helpers = new LinkedHashMap<OneDoFJoint, DiagnosticsWhenHangingHelper>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> tauOutput = new LinkedHashMap<>();
   private final FullRobotModel fullRobotModel;
   private final LogDataProcessorHelper logDataProcessorHelper;

   public DiagnosticLogProcessorFunction(LogDataProcessorHelper logDataProcessorHelper)
   {
      this.logDataProcessorHelper = logDataProcessorHelper;
      fullRobotModel = logDataProcessorHelper.getFullRobotModel();
      new FullRobotModelCorruptor("diagnostic", fullRobotModel, registry);
      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      for (RobotSide robotSide : RobotSide.values)
      {
         makeLegJointHelper(robotSide, false, LegJointName.HIP_YAW);
         makeLegJointHelper(robotSide, true, LegJointName.HIP_PITCH);
         makeLegJointHelper(robotSide, false, LegJointName.HIP_ROLL);
         makeLegJointHelper(robotSide, true, LegJointName.KNEE);
         makeLegJointHelper(robotSide, true, LegJointName.ANKLE_PITCH);
         makeLegJointHelper(robotSide, false, LegJointName.ANKLE_ROLL);

         makeArmJointHelper(robotSide, true, ArmJointName.SHOULDER_PITCH);
         makeArmJointHelper(robotSide, false, ArmJointName.SHOULDER_ROLL);
         makeArmJointHelper(robotSide, false, ArmJointName.SHOULDER_YAW);
         makeArmJointHelper(robotSide, true, ArmJointName.ELBOW_PITCH);
      }

      SideDependentList<InverseDynamicsJoint> topLegJoints = new SideDependentList<InverseDynamicsJoint>();
      for (RobotSide robotSide : RobotSide.values())
      {
         topLegJoints.set(robotSide, fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW));
      }

      OneDoFJoint spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, false, true, topLegJoints, registry));

      spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, true, true, topLegJoints, registry));

      spineJoint = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      helpers.put(spineJoint, new DiagnosticsWhenHangingHelper(spineJoint, false, true, topLegJoints, registry));
      
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DoubleYoVariable tau = new DoubleYoVariable("tau_diag_" + oneDoFJoint.getName(), registry);
         tauOutput.put(oneDoFJoint, tau);
      }
   }

   private void makeArmJointHelper(RobotSide robotSide, boolean preserveY, ArmJointName armJointName)
   {
      OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
      helpers.put(armJoint, new DiagnosticsWhenHangingHelper(armJoint, preserveY, registry));
   }

   private void makeLegJointHelper(RobotSide robotSide, boolean preserveY, LegJointName legJointName)
   {
      OneDoFJoint legJoint = fullRobotModel.getLegJoint(robotSide, legJointName);
      helpers.put(legJoint, new DiagnosticsWhenHangingHelper(legJoint, preserveY, registry));
   }

   @Override
   public void processDataAtStateEstimatorRate()
   {
   }

   @Override
   public void processDataAtControllerRate()
   {
      logDataProcessorHelper.update();
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = helpers.get(oneDoFJoint);
         if (diagnosticsWhenHangingHelper != null)
         {
            diagnosticsWhenHangingHelper.update();
         
            if (tauOutput.get(oneDoFJoint) != null)
               tauOutput.get(oneDoFJoint).set(diagnosticsWhenHangingHelper.getEstimatedTorque() + diagnosticsWhenHangingHelper.getTorqueOffset());
         }
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }
}
