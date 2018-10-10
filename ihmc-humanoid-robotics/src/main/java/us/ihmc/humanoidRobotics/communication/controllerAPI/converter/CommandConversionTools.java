package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;

public class CommandConversionTools
{
   public static void convertToSE3(SO3TrajectoryControllerCommand command, SE3TrajectoryControllerCommand commandToPack)
   {
      commandToPack.setQueueableCommandVariables(command);
      commandToPack.setTrajectoryFrame(command.getTrajectoryFrame());
      commandToPack.getSelectionMatrix().clearLinearSelection();
      commandToPack.getSelectionMatrix().setAngularPart(command.getSelectionMatrix());
      commandToPack.getWeightMatrix().clearLinearWeights();
      commandToPack.getWeightMatrix().setAngularPart(command.getWeightMatrix());
      commandToPack.setUseCustomControlFrame(command.useCustomControlFrame());
      commandToPack.setControlFramePose(command.getControlFramePose());
      commandToPack.getTrajectoryPointList().setToOrientationTrajectoryIncludingFrame(command.getTrajectoryPointList());
   }

   public static void convertToSE3(EuclideanTrajectoryControllerCommand command, SE3TrajectoryControllerCommand commandToPack)
   {
      commandToPack.setQueueableCommandVariables(command);
      commandToPack.setTrajectoryFrame(command.getTrajectoryFrame());
      commandToPack.getSelectionMatrix().clearAngularSelection();
      commandToPack.getSelectionMatrix().setLinearPart(command.getSelectionMatrix());
      commandToPack.getWeightMatrix().clearAngularWeights();
      commandToPack.getWeightMatrix().setLinearPart(command.getWeightMatrix());
      commandToPack.setUseCustomControlFrame(command.useCustomControlFrame());
      commandToPack.setControlFramePose(command.getControlFramePose());
      commandToPack.getTrajectoryPointList().setToPositionTrajectoryIncludingFrame(command.getTrajectoryPointList());
   }

   public static void convertToSO3(SE3TrajectoryControllerCommand command, SO3TrajectoryControllerCommand commandToPack)
   {
      commandToPack.setQueueableCommandVariables(command);
      commandToPack.setTrajectoryFrame(command.getTrajectoryFrame());
      commandToPack.getSelectionMatrix().set(command.getSelectionMatrix().getAngularPart());
      commandToPack.getWeightMatrix().set(command.getWeightMatrix().getAngularPart());
      commandToPack.setUseCustomControlFrame(command.useCustomControlFrame());
      commandToPack.setControlFramePose(command.getControlFramePose());
      commandToPack.getTrajectoryPointList().setIncludingFrame(command.getTrajectoryPointList());
   }

   public static void convertToEuclidean(SE3TrajectoryControllerCommand command, EuclideanTrajectoryControllerCommand commandToPack)
   {
      commandToPack.setQueueableCommandVariables(command);
      commandToPack.setTrajectoryFrame(command.getTrajectoryFrame());
      commandToPack.getSelectionMatrix().set(command.getSelectionMatrix().getLinearPart());
      commandToPack.getWeightMatrix().set(command.getWeightMatrix().getLinearPart());
      commandToPack.setUseCustomControlFrame(command.useCustomControlFrame());
      commandToPack.setControlFramePose(command.getControlFramePose());
      commandToPack.getTrajectoryPointList().setIncludingFrame(command.getTrajectoryPointList());
   }
}
