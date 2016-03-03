package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePauseWalkingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.output.ControllerStatusOutputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class WalkingMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // TODO Need to find something better than an ArrayList.
   private final List<Footstep> upcomingFootsteps = new ArrayList<>();
   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private ModifiableFootTrajectoryMessage nextFootTrajectoryForFlamingoStance;

   private final ControllerStatusOutputManager statusOutputManager;

   private final IntegerYoVariable currentFootstepIndex = new IntegerYoVariable("currentFootstepIndex", registry);
   private final IntegerYoVariable currentNumberOfFootsteps = new IntegerYoVariable("currentNumberOfFootsteps", registry);
   private final BooleanYoVariable isWalkingPaused = new BooleanYoVariable("isWalkingPaused", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("transferTime", registry);
   private final DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", registry);

   private final int numberOfFootstepsToVisualize = 4;
   @SuppressWarnings("unchecked")
   private final EnumYoVariable<RobotSide>[] upcomingFoostepSide = new EnumYoVariable[numberOfFootstepsToVisualize];

   public WalkingMessageHandler(double defaultTransferTime, double defaultSwingTime, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         ControllerStatusOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      this.contactableFeet = contactableFeet;
      this.statusOutputManager = statusOutputManager;

      transferTime.set(defaultTransferTime);
      swingTime.set(defaultSwingTime);

      for (int i = 0; i < numberOfFootstepsToVisualize; i++)
         upcomingFoostepSide[i] = new EnumYoVariable<>("upcomingFoostepSide" + i, registry, RobotSide.class, true);
      updateVisualization();

      parentRegistry.addChild(registry);
   }

   public void handleFootstepDataListMessage(ModifiableFootstepDataListMessage message)
   {
      if (message.getNumberOfFootsteps() > 0)
      {
         upcomingFootsteps.clear();
         currentFootstepIndex.set(0);
         isWalkingPaused.set(false);
         clearFootTrajectory();
      }

      double messageTransferTime = message.getTransferTime();
      double messageSwingTime = message.getSwingTime();
      if (!Double.isNaN(messageSwingTime) && messageSwingTime > 1.0e-2 && !Double.isNaN(messageTransferTime) && messageTransferTime > 1.0e-2)
      {
         transferTime.set(messageTransferTime);
         swingTime.set(messageSwingTime);
      }

      currentNumberOfFootsteps.set(message.getNumberOfFootsteps());

      for (int i = 0; i < message.getNumberOfFootsteps(); i++)
      {
         Footstep newFootstep = createFootstep(message.getFootstep(i));
         upcomingFootsteps.add(newFootstep);
      }
      updateVisualization();
   }

   public void handlePauseWalkingMessage(ModifiablePauseWalkingMessage message)
   {
      isWalkingPaused.set(message.isPauseRequested());
   }

   public void handleFootTrajectoryMessage(ModifiableFootTrajectoryMessage message)
   {
      nextFootTrajectoryForFlamingoStance = message;
   }

   public Footstep peek(int i)
   {
      if (i >= upcomingFootsteps.size())
         return null;
      else
         return upcomingFootsteps.get(i);
   }

   public Footstep poll()
   {
      if (upcomingFootsteps.isEmpty())
         return null;
      else
      {
         updateVisualization();
         currentNumberOfFootsteps.decrement();
         currentFootstepIndex.increment();
         return upcomingFootsteps.remove(0);
      }
   }

   public ModifiableFootTrajectoryMessage pollFootTrajectoryForFlamingoStance()
   {
      ModifiableFootTrajectoryMessage ret = nextFootTrajectoryForFlamingoStance;
      nextFootTrajectoryForFlamingoStance = null;
      return ret;
   }

   public ModifiableFootTrajectoryMessage pollFootTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      if (!hasFootTrajectoryForFlamingoStance(swingSide))
         return null;
      else
         return pollFootTrajectoryForFlamingoStance();
   }

   public void insertNextFootstep(Footstep newNextFootstep)
   {
      if (newNextFootstep != null)
         upcomingFootsteps.add(0, newNextFootstep);
   }

   public boolean hasUpcomingFootsteps()
   {
      return !upcomingFootsteps.isEmpty() && !isWalkingPaused.getBooleanValue();
   }

   public boolean isNextFootstepFor(RobotSide swingSide)
   {
      if (!hasUpcomingFootsteps())
         return false;
      else
         return peek(0).getRobotSide() == swingSide;
   }

   public boolean hasFootTrajectoryForFlamingoStance()
   {
      return nextFootTrajectoryForFlamingoStance != null;
   }

   public boolean hasFootTrajectoryForFlamingoStance(RobotSide swingSide)
   {
      return hasFootTrajectoryForFlamingoStance() && nextFootTrajectoryForFlamingoStance.getRobotSide() == swingSide;
   }

   public boolean isWalkingPaused()
   {
      return isWalkingPaused.getBooleanValue();
   }

   public void clearFootTrajectory()
   {
      nextFootTrajectoryForFlamingoStance = null;
   }

   public void clearFootsteps()
   {
      upcomingFootsteps.clear();
      currentNumberOfFootsteps.set(0);
      currentFootstepIndex.set(0);
      updateVisualization();
   }

   private final Point3d actualFootPositionInWorld = new Point3d();
   private final Quat4d actualFootOrientationInWorld = new Quat4d();

   public void reportFootstepStarted(RobotSide robotSide)
   {
      statusOutputManager.reportStatusMessage(new FootstepStatus(FootstepStatus.Status.STARTED, currentFootstepIndex.getIntegerValue()));
   }

   public void reportFootstepCompleted(RobotSide robotSide, FramePose actualFootPoseInWorld)
   {
      actualFootPoseInWorld.getPose(actualFootPositionInWorld, actualFootOrientationInWorld);
      statusOutputManager.reportFootstepStatus(new FootstepStatus(FootstepStatus.Status.COMPLETED, currentFootstepIndex.getIntegerValue(),
            actualFootPositionInWorld, actualFootOrientationInWorld, robotSide));
   }

   public void reportWalkingComplete()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.COMPLETED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
   }

   public void reportWalkingAbortRequested()
   {
      WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
      walkingStatusMessage.setWalkingStatus(WalkingStatusMessage.Status.ABORT_REQUESTED);
      statusOutputManager.reportStatusMessage(walkingStatusMessage);
   }

   public double getTransferTime()
   {
      return transferTime.getDoubleValue();
   }

   public double getSwingTime()
   {
      return swingTime.getDoubleValue();
   }

   public double getStepTime()
   {
      return transferTime.getDoubleValue() + swingTime.getDoubleValue();
   }

   private void updateVisualization()
   {
      for (int i = 0; i < upcomingFootsteps.size(); i++)
      {
         if (i < numberOfFootstepsToVisualize)
            upcomingFoostepSide[i].set(upcomingFootsteps.get(i).getRobotSide());
      }

      for (int i = upcomingFootsteps.size(); i < numberOfFootstepsToVisualize; i++)
      {
         upcomingFoostepSide[i].set(null);
      }
   }

   private Footstep createFootstep(ModifiableFootstepDataMessage footstepData)
   {
      FramePose footstepPose = new FramePose(worldFrame, footstepData.getPosition(), footstepData.getOrientation());
      PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", footstepPose);

      List<Point2d> contactPoints;
      if (footstepData.getPredictedContactPoints().isEmpty())
         contactPoints = null;
      else
      {
         contactPoints = new ArrayList<>();
         for (int i = 0; i < footstepData.getPredictedContactPoints().size(); i++)
            contactPoints.add(footstepData.getPredictedContactPoints().get(i));
      }

      RobotSide robotSide = footstepData.getRobotSide();
      TrajectoryType trajectoryType = footstepData.getTrajectoryType();

      ContactablePlaneBody contactableFoot = contactableFeet.get(robotSide);
      ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
      RigidBody rigidBody = contactableFoot.getRigidBody();

      Footstep footstep = new Footstep(rigidBody, robotSide, soleFrame, footstepPoseFrame, true, contactPoints);

      footstep.trajectoryType = trajectoryType;
      footstep.swingHeight = footstepData.getSwingHeight();
      switch (footstepData.getOrigin())
      {
      case AT_ANKLE_FRAME:
         break;
      case AT_SOLE_FRAME:
         footstep.setSolePose(footstepPose);
         break;
      default:
         throw new RuntimeException("Should not get there.");
      }
      return footstep;
   }
}
