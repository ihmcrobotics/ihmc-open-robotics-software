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
      gains.setKp(tools.getParameters().getOutputFeedbackGain());
      gains.setZeta(tools.getParameters().getOutputFeedbackDampingRatio());

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

   private final Quaternion diff = new Quaternion();
   private final SpatialVector feedbackPosition = new SpatialVector();
   private final SpatialVector feedbackVelocity = new SpatialVector();

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
         double qdMin = outputRobotState.getOneDoFJoints()[i].getVelocityLimitLower();
         double qdMax = outputRobotState.getOneDoFJoints()[i].getVelocityLimitUpper();

         double qDDotMin = KSTTools.computeJointMinAcceleration(qMin,
                                                                qdMin,
                                                                outputRobotState.getJointPosition(i),
                                                                outputRobotState.getJointVelocity(i),
                                                                updateDT);
         if (Double.isFinite(qDDotMin))
            qdd = Math.max(qdd, qDDotMin);

         double qDDotMax = KSTTools.computeJointMaxAcceleration(qMax,
                                                                qdMax,
                                                                outputRobotState.getJointPosition(i),
                                                                outputRobotState.getJointVelocity(i),
                                                                updateDT);
         if (Double.isFinite(qDDotMax))
            qdd = Math.min(qdd, qDDotMax);

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
