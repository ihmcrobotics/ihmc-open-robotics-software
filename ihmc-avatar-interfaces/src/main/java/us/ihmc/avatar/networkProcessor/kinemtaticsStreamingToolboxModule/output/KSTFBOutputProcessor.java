package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.yoVariables.registry.YoRegistry;

public class KSTFBOutputProcessor implements KSTOutputProcessor
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoPDGains gains;
   private final YoKinematicsToolboxOutputStatus outputRobotState;

   private final double updateDT;

   public KSTFBOutputProcessor(KSTTools tools, YoRegistry registry)
   {
      gains = new YoPDGains("Output", registry);
      gains.createDerivativeGainUpdater(false);
      gains.setKp(500.0);
      gains.setZeta(1.0);

      updateDT = tools.getToolboxControllerPeriod();

      outputRobotState = new YoKinematicsToolboxOutputStatus("feedback", tools.getDesiredFullRobotModel(), registry);
      outputRobotState.createAccelerationState();
   }

   private boolean firstTick = true;

   @Override
   public void initialize()
   {
      outputRobotState.setToNaN();
      firstTick = true;
   }

   @Override
   public void update(double time, boolean wasStreaming, boolean isStreaming, KSTOutputDataReadOnly latestOutput)
   {
      if (firstTick)
      {
         outputRobotState.set(latestOutput);
         firstTick = false;
         return;
      }

      outputRobotState.checkCompatibility(latestOutput);
      Quaternion diff = new Quaternion();
      SpatialVector feedbackPosition = new SpatialVector();
      SpatialVector feedbackVelocity = new SpatialVector();

      feedbackPosition.getLinearPart().sub(latestOutput.getRootJointPosition(), outputRobotState.getRootJointPosition());
      outputRobotState.getRootJointOrientation().inverseTransform(feedbackPosition.getLinearPart());
      diff.difference(outputRobotState.getRootJointOrientation(), latestOutput.getRootJointOrientation());
      diff.normalizeAndLimitToPi();
      diff.getRotationVector(feedbackPosition.getAngularPart());
      feedbackPosition.scale(gains.getKp());

      feedbackVelocity.getLinearPart().sub(latestOutput.getRootJointLinearVelocity(), outputRobotState.getRootJointLinearVelocity());
      feedbackVelocity.getAngularPart().sub(latestOutput.getRootJointAngularVelocity(), outputRobotState.getRootJointAngularVelocity());
      feedbackVelocity.scale(gains.getKd());

      outputRobotState.getRootJointLinearAcceleration().add(feedbackPosition.getLinearPart(), feedbackVelocity.getLinearPart());
      outputRobotState.getRootJointAngularAcceleration().add(feedbackPosition.getAngularPart(), feedbackVelocity.getAngularPart());

      for (int i = 0; i < outputRobotState.getNumberOfJoints(); i++)
      {
         double q_err = latestOutput.getJointPosition(i) - outputRobotState.getJointPosition(i);
         double qd_err = latestOutput.getJointVelocity(i) - outputRobotState.getJointVelocity(i);
         double qdd = gains.getKp() * q_err + gains.getKd() * qd_err;

         double qMin = outputRobotState.getOneDoFJoints()[i].getJointLimitLower();
         double qMax = outputRobotState.getOneDoFJoints()[i].getJointLimitUpper();

         if (!Double.isInfinite(qMin))
         {
            double qDDotMin = KSTTools.computeJointMinAcceleration(qMin, outputRobotState.getJointPosition(i), outputRobotState.getJointVelocity(i), updateDT);
            qdd = Math.max(qdd, qDDotMin);
         }

         if (!Double.isInfinite(qMax))
         {
            double qDDotMax = KSTTools.computeJointMaxAcceleration(qMax, outputRobotState.getJointPosition(i), outputRobotState.getJointVelocity(i), updateDT);
            qdd = Math.min(qdd, qDDotMax);
         }

         outputRobotState.setJointAcceleration(i, qdd);
      }

      outputRobotState.integrate(updateDT);
   }

   @Override
   public KSTOutputDataReadOnly getProcessedOutput()
   {
      return outputRobotState;
   }
}
