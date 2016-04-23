package us.ihmc.quadrupedRobotics;

import java.util.ArrayList;
import java.util.Map;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.controllers.PDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedOutputWriterWithControlModeSelection implements OutputWriter
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final SDFPerfectSimulatedOutputWriter outputWriter;

   private final ArrayList<PDPositionControllerForOneDoFJoint> positionControllers = new ArrayList<>();

   private final SDFFullRobotModel sdfFullRobotModel;

   public SimulatedOutputWriterWithControlModeSelection(SDFFullRobotModel sdfFullRobotModel, SDFRobot robot, YoVariableRegistry parentRegistry,
         Map<OneDoFJoint, PDGainsInterface> gains)
   {
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.outputWriter = new SDFPerfectSimulatedOutputWriter(robot, sdfFullRobotModel);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      createPDControllers(sdfFullRobotModel, oneDegreeOfFreedomJoints, gains);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      outputWriter.setFullRobotModel(sdfFullRobotModel);
   }

   @Override
   public void write()
   {
      for (int i = 0; i < positionControllers.size(); i++)
      {
         if (positionControllers.get(i).doPositionControl())
         {
            positionControllers.get(i).update();
         }
      }

      outputWriter.write();
   }

   private void createPDControllers(SDFFullRobotModel sdfFullRobotModel, ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints,
         Map<OneDoFJoint, PDGainsInterface> gains)
   {
      for (OneDegreeOfFreedomJoint simulatedJoint : oneDegreeOfFreedomJoints)
      {
         String jointName = simulatedJoint.getName();
         OneDoFJoint oneDoFJoint = sdfFullRobotModel.getOneDoFJointByName(jointName);
         PDGainsInterface jointGains = gains.get(oneDoFJoint);

         double kp = jointGains == null ? 0.0 : jointGains.getKp();
         double kd = jointGains == null ? 0.0 : jointGains.getKd();
         double maxTorque = simulatedJoint.getTorqueLimit();

         positionControllers.add(new PDPositionControllerForOneDoFJoint(oneDoFJoint, kp, kd, maxTorque));
      }
   }

   public class PDPositionControllerForOneDoFJoint
   {
      private final PDController pdController;
      private final YoVariableRegistry pidRegistry;
      private final OneDoFJoint oneDofJoint;
      private final DoubleYoVariable q_d, q_d_notCapped, tau_d, tau_d_notCapped, maxTorque;
      private final BooleanYoVariable hitJointLimit, hitTorqueLimit;

      public PDPositionControllerForOneDoFJoint(OneDoFJoint oneDofJoint, double kp, double kd, double torqueLimit)
      {
         String name = "pdController_" + oneDofJoint.getName();
         pidRegistry = new YoVariableRegistry(name);
         q_d = new DoubleYoVariable(name + "_q_d", pidRegistry);
         q_d_notCapped = new DoubleYoVariable(name + "_q_d_notCapped", pidRegistry);
         tau_d = new DoubleYoVariable(name + "_tau_d", pidRegistry);
         tau_d_notCapped = new DoubleYoVariable(name + "_tau_d_notCapped", pidRegistry);
         maxTorque = new DoubleYoVariable(name + "_tau_max", pidRegistry);
         maxTorque.set(torqueLimit);
         hitJointLimit = new BooleanYoVariable(name + "_hitJointLimit", pidRegistry);
         hitTorqueLimit = new BooleanYoVariable(name + "_hitTorqueLimit", pidRegistry);

         pdController = new PDController(oneDofJoint.getName(), pidRegistry);
         pdController.setProportionalGain(kp);
         pdController.setDerivativeGain(kd);
         registry.addChild(pidRegistry);

         this.oneDofJoint = oneDofJoint;
      }

      public void update()
      {
         double currentPosition = oneDofJoint.getQ();
         double desiredPosition = oneDofJoint.getqDesired();

         q_d_notCapped.set(desiredPosition);
         double desiredPositionClipped = MathTools.clipToMinMax(desiredPosition, oneDofJoint.getJointLimitLower(), oneDofJoint.getJointLimitUpper());
         boolean insidePosiotionLimits = MathTools.isInsideBoundsInclusive(desiredPosition, oneDofJoint.getJointLimitLower(), oneDofJoint.getJointLimitUpper());
         hitJointLimit.set(!insidePosiotionLimits);
         q_d.set(desiredPositionClipped);

         double currentRate = oneDofJoint.getQd();
         double desiredRate = oneDofJoint.getQdDesired();
         double desiredTau = pdController.compute(currentPosition, desiredPositionClipped, currentRate, desiredRate);

         tau_d_notCapped.set(desiredTau);
         boolean insideTauLimits = MathTools.isInsideBoundsInclusive(desiredTau, -maxTorque.getDoubleValue(), maxTorque.getDoubleValue());
         hitTorqueLimit.set(!insideTauLimits);
         desiredTau = MathTools.clipToMinMax(desiredTau, maxTorque.getDoubleValue());

         tau_d.set(desiredTau);
         oneDofJoint.setTau(desiredTau);
      }

      public boolean doPositionControl()
      {
         return oneDofJoint.isUnderPositionControl();
      }
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      outputWriter.setFullRobotModel(sdfFullRobotModel);
   }
}
