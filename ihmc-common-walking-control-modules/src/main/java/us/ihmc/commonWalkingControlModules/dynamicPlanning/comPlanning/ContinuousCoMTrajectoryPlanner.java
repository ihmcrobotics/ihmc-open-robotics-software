package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class ContinuousCoMTrajectoryPlanner implements CoMTrajectoryPlannerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;
   private final List<? extends ContactStateProvider> contactSequence;

   private final FrameTrajectory3D transitionTrajectory = new FrameTrajectory3D(6, ReferenceFrame.getWorldFrame());

   public ContinuousCoMTrajectoryPlanner(CoMTrajectoryPlannerInterface comTrajectoryPlanner, List<? extends ContactStateProvider> contactSequence,
                                         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.comTrajectoryPlanner = comTrajectoryPlanner;
      this.contactSequence = contactSequence;

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      comTrajectoryPlanner.setNominalCoMHeight(nominalCoMHeight);
   }

   /** {@inheritDoc} */
   @Override
   public void solveForTrajectory()
   {
      comTrajectoryPlanner.solveForTrajectory();
   }

   /** {@inheritDoc} */
   @Override
   public void compute(double timeInPhase)
   {
      comTrajectoryPlanner.compute(timeInPhase);
   }

   /** {@inheritDoc} */
   @Override
   public void setCurrentCoMPosition(FramePoint3DReadOnly currentCoMPosition)
   {
      comTrajectoryPlanner.setCurrentCoMPosition(currentCoMPosition);
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return comTrajectoryPlanner.getDesiredDCMPosition();
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return comTrajectoryPlanner.getDesiredDCMVelocity();
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return comTrajectoryPlanner.getDesiredCoMPosition();
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return comTrajectoryPlanner.getDesiredCoMVelocity();
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return comTrajectoryPlanner.getDesiredCoMAcceleration();
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }
}
