package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedContactSequenceUpdater;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedTimedStep;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
//us.ihmc.euclid.tools.EuclidCoreIOTools.TimeInterval
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the simple Version of the BipedSimpleCoMTrajectoryPlanner: TODO: Update following explanation
 * Change from taking in {@link BipedTimedStep} to taking in {@link footstep} and {@link footsteptiming}
 * 
 * a base class for bipeds for dynamic trajectory planning. It is used to generate feasible DCM, CoM, and VRP trajectories. The inputs to this class
 * are a list of {@link BipedTimedStep}, which are converted to a list of {@link ContactStateProvider}, which is then used by the {@link CoMTrajectoryPlanner}.
 * This is done using {@link BipedContactSequenceUpdater} class.
 *
 * <p>
 * WARNING: This class current generates garbage from the {@link BipedContactSequenceTools#computeStepTransitionsFromStepSequence}.
 * </p>
 */

/*
 * 1. instantiated with a sideDependentList of SoleFrames (  ), gravity, nominal height, omega0, parent Registry, and yographics registry
 * 2. Add footsteps (footstep and footsteptiming) to the step sequence
 * 3. List of contact state providers created for the SimpleBipedCoMTrajectoryPlanner
 * 4. Solve for trajectory in the planner
 * 5. Compute the planner to get the desired vrp and such
 */
public class SimpleBipedCoMTrajectoryPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SimpleBipedContactSequenceUpdater sequenceUpdater;
   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;

   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);
   
   private final List<Footstep> footstepList = new ArrayList<>();
   private final List<FootstepTiming> footstepTimingList = new ArrayList<>();
     
   //Sole frames pass in the reference frames attached to the center of the bottom of the foot. Can be used to get the current 
   public SimpleBipedCoMTrajectoryPlanner(SideDependentList<MovingReferenceFrame> soleFrames, double gravityZ, double nominalCoMHeight,
                                          DoubleProvider omega0, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      sequenceUpdater = new SimpleBipedContactSequenceUpdater(soleFrames, registry, yoGraphicsListRegistry);
      comTrajectoryPlanner = new SimpleCoMTrajectoryPlanner(omega0);
      comTrajectoryPlanner.setNominalCoMHeight(nominalCoMHeight);
      ((SimpleCoMTrajectoryPlanner) comTrajectoryPlanner).setCornerPointViewer(new CornerPointViewer(registry, yoGraphicsListRegistry));

      parentRegistry.addChild(registry);
   }
   
   public void addStepToSequence(Footstep footstep, FootstepTiming timing, FootstepShiftFractions shiftFractions, double CurrentTime)
   {
      footstepList.add(footstep);
      footstepTimingList.add(timing);
   }
   
   public void clearStepSequence()
   {
      footstepList.clear();
      footstepTimingList.clear();
   }

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void initialize()
   {
      sequenceUpdater.initialize();
   }

   public void computeSetpoints(double currentTime, List<RobotSide> currentFeetInContact)
   {
      sequenceUpdater.update(footstepList, footstepTimingList, currentFeetInContact, currentTime);

      double timeInPhase = currentTime - sequenceUpdater.getAbsoluteContactSequence().get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      comTrajectoryPlanner.solveForTrajectory(sequenceUpdater.getContactSequence());
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return comTrajectoryPlanner.getDesiredDCMPosition();
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return comTrajectoryPlanner.getDesiredDCMVelocity();
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return comTrajectoryPlanner.getDesiredCoMPosition();
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return comTrajectoryPlanner.getDesiredCoMVelocity();
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return comTrajectoryPlanner.getDesiredCoMAcceleration();
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }

   public List<Trajectory3D> getVRPTrajectories()
   {
      return ((SimpleCoMTrajectoryPlanner) comTrajectoryPlanner).getVRPTrajectories();
   }
   
   public List<Footstep> getFootstepList()
   {
      return footstepList;
   }
   
   public List<FootstepTiming> getFootstepTimingList()
   {
      return footstepTimingList;
   }
}

