package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WrenchTrajectoryControllerCommand;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.robotics.math.trajectories.LinearSpatialVectorTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class ExternalForceCommandHandler
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultipleWaypointsPositionTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("externalForce", ReferenceFrame.getWorldFrame(), registry);

   private final SpatialVector tempWrench = new SpatialVector();
   private final FramePoint3D tempForce = new FramePoint3D();

   private final DoubleProvider yoTime;
   private final double mass;
   private final double massInverse;

   public ExternalForceCommandHandler(DoubleProvider yoTime, double mass, YoRegistry parentRegistry)
   {
      this.yoTime = yoTime;
      this.mass = mass;
      this.massInverse = 1.0 / mass;

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      trajectoryGenerator.clear();
   }

   public void addTrajectory(WrenchTrajectoryControllerCommand command)
   {
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE)
         trajectoryGenerator.clear();

      double executionTime = command.getExecutionTime();
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         double trajectoryTime = command.getTrajectoryPointTime(i);

         tempWrench.setIncludingFrame(command.getTrajectoryPoint(i));
         tempWrench.changeFrame(ReferenceFrame.getWorldFrame());

         tempForce.set(tempWrench.getLinearPart());

         // FIXME make this linear
         trajectoryGenerator.appendWaypoint(trajectoryTime + executionTime, tempForce, new FrameVector3D());
      }
   }

   public void addToDesiredContactState(List<SettableContactStateProvider> contactStateProviders)
   {
      if (trajectoryGenerator.isEmpty() || trajectoryGenerator.isDone())
         return;

      double timeForCompute = yoTime.getValue();
      trajectoryGenerator.compute(timeForCompute);
      tempForce.setIncludingFrame(trajectoryGenerator.getPosition());
      tempForce.scale(massInverse);

      contactStateProviders.get(0).setExternalContactAccelerationStart(tempForce);

      timeForCompute += contactStateProviders.get(0).getTimeInterval().getDuration();
      trajectoryGenerator.compute(timeForCompute);
      tempForce.setAndScale(massInverse, trajectoryGenerator.getPosition());

      contactStateProviders.get(0).setExternalContactAccelerationEnd(tempForce);

      for (int i = 1; i < contactStateProviders.size(); i++)
      {
         contactStateProviders.get(i).setExternalContactAccelerationStart(tempForce);

         timeForCompute += contactStateProviders.get(i).getTimeInterval().getDuration();
         trajectoryGenerator.compute(timeForCompute);
         tempForce.setAndScale(massInverse, trajectoryGenerator.getPosition());

         contactStateProviders.get(1).setExternalContactAccelerationEnd(tempForce);
      }
   }
}
