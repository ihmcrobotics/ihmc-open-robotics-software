package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerIndexHandler;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.MultipleWrenchSegmentTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class WrenchMPCTrajectoryHandler
{
   protected final MultipleWrenchSegmentTrajectoryGenerator wrenchTrajectory;

   private boolean hasTrajectory = false;

   public WrenchMPCTrajectoryHandler(YoRegistry registry)
   {
      wrenchTrajectory = new MultipleWrenchSegmentTrajectoryGenerator("desiredWrenchTrajectory", ReferenceFrame.getWorldFrame(), registry);
   }

   public void clearTrajectory()
   {
      wrenchTrajectory.clear();
      hasTrajectory = false;
   }

   public boolean hasTrajectory()
   {
      return hasTrajectory;
   }

   public void extractSolutionForPreviewWindow(List<PreviewWindowSegment> planningWindow,
                                               List<? extends List<MPCContactPlane>> contactPlaneHelpers,
                                               double mass,
                                               double omega)
   {
      clearTrajectory();

      for (int i = 0; i < planningWindow.size(); i++)
      {
         TimeIntervalReadOnly timeInterval = planningWindow.get(i);
         wrenchTrajectory.appendSegment(timeInterval, mass, omega, contactPlaneHelpers.get(i));
      }

      wrenchTrajectory.initialize();
      hasTrajectory = true;
   }

   public void compute(double timeInPhase)
   {
      wrenchTrajectory.compute(timeInPhase);
   }

   public MultipleWrenchSegmentTrajectoryGenerator getWrenchTrajectory()
   {
      return wrenchTrajectory;
   }

   public WrenchReadOnly getDesiredWrench()
   {
      return wrenchTrajectory.getSegment(wrenchTrajectory.getCurrentSegmentIndex()).getWrench();
   }
}
