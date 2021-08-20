package us.ihmc.commonWalkingControlModules.modelPredictiveController.customPolicies;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.CoMPositionCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.DCMPositionCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.PreviewWindowSegment;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

import java.util.List;

public class CustomDCMPositionPolicy implements CustomMPCPolicy
{
   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private double timeOfPolicy;
   private double weight;

   private final DCMPositionCommand mpcCommand = new DCMPositionCommand();

   public FramePoint3DBasics getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   public void setDesiredDCMPosition(FramePoint3DReadOnly desiredDCMPosition)
   {
      this.desiredDCMPosition.setIncludingFrame(desiredDCMPosition);
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setTimeOfPolicy(double timeOfPolicy)
   {
      this.timeOfPolicy = timeOfPolicy;
   }

   public void setPolicyWeight(double weight)
   {
      this.weight = weight;
   }

   @Override
   public MPCCommand<?> computeMPCCommand(MPCContactHandler contactHandler, List<PreviewWindowSegment> contactStateProviders, double omega)
   {
      int segmentNumber = CustomPolicyTools.getSegmentNumber(timeOfPolicy, contactStateProviders);

      if (segmentNumber < 0)
         return null;

      double timeInSegment = CustomPolicyTools.getTimeInSegment(segmentNumber, timeOfPolicy, contactStateProviders);
      timeInSegment = Math.min(timeInSegment, CoMTrajectoryPlannerTools.sufficientlyLongTime);

      mpcCommand.clear();
      mpcCommand.setSegmentNumber(segmentNumber);
      mpcCommand.setTimeOfObjective(timeInSegment);
      mpcCommand.setObjective(desiredDCMPosition);
      mpcCommand.setWeight(weight);
      mpcCommand.setOmega(omega);
      mpcCommand.setConstraintType(ConstraintType.OBJECTIVE);
      mpcCommand.getSelectionMatrix().set(selectionMatrix);
      for (int i = 0; i < contactHandler.getNumberOfContactPlanesInSegment(segmentNumber); i++)
         mpcCommand.addContactPlaneHelper(contactHandler.getContactPlane(segmentNumber, i));

      return mpcCommand;
   }

   @Override
   public MPCCommand<?> getMPCCommand()
   {
      return mpcCommand;
   }
}
