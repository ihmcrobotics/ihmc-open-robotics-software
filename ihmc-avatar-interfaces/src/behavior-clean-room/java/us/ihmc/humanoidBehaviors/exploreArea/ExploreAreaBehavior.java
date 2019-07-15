package us.ihmc.humanoidBehaviors.exploreArea;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class ExploreAreaBehavior
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final List<Double> chestYawsForLookingAround = Arrays.asList(-10.0, -20.0, -30.0, 0.0);
   private final List<Point3D> pointsObservedFrom = new ArrayList<Point3D>();

   private final double turnChestTrajectoryDuration = 3.0;
   private final double perceiveDuration = 4.0;

   private int chestYawForLookingAroundIndex = 0;

   public enum ExploreAreaBehaviorState
   {
      Stop, LookAround, Perceive, GrabPlanarRegions
   }

   private final BehaviorHelper behaviorHelper;

   private final Messager messager;
   private final StateMachine<ExploreAreaBehaviorState, State> stateMachine;

   private final AtomicReference<Boolean> explore;

   private PlanarRegionsList concatenatedMap;

   public ExploreAreaBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel)
   {
      this.behaviorHelper = behaviorHelper;
      this.messager = messager;

      explore = messager.createInput(ExploreAreaBehaviorAPI.ExploreArea, false);

      LogTools.debug("Initializing patrol behavior");

      EnumBasedStateMachineFactory<ExploreAreaBehaviorState> factory = new EnumBasedStateMachineFactory<>(ExploreAreaBehaviorState.class);
      factory.setOnEntry(ExploreAreaBehaviorState.Stop, this::onStopStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.Stop, this::doStopStateAction);
      factory.setOnEntry(ExploreAreaBehaviorState.LookAround, this::onLookAroundStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.LookAround, this::doLookAroundStateAction);
      factory.setOnEntry(ExploreAreaBehaviorState.Perceive, this::onPerceiveStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.Perceive, this::doPerceiveStateAction);
      factory.setOnEntry(ExploreAreaBehaviorState.GrabPlanarRegions, this::onGrabPlanarRegionsStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.GrabPlanarRegions, this::doGrabPlanarRegionsStateAction);

      factory.addTransition(ExploreAreaBehaviorState.Stop, ExploreAreaBehaviorState.LookAround, this::readyToTransitionFromStopToLookAround);
      factory.addTransition(ExploreAreaBehaviorState.LookAround, ExploreAreaBehaviorState.Perceive, this::readyToTransitionFromLookAroundToPerceive);
      factory.addTransition(ExploreAreaBehaviorState.Perceive,
                            ExploreAreaBehaviorState.GrabPlanarRegions,
                            this::readyToTransitionFromPerceiveToGrabPlanarRegions);

      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions, ExploreAreaBehaviorState.Stop, this::readyToTransitionFromGrabPlanarRegionsToStop);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions,
                            ExploreAreaBehaviorState.LookAround,
                            this::readyToTransitionFromGrabPlanarRegionsToLookAround);

      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(ExploreAreaBehaviorState.Stop);

      ExceptionHandlingThreadScheduler exploreAreaThread = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                DefaultExceptionHandler.PRINT_STACKTRACE,
                                                                                                5);
      exploreAreaThread.schedule(this::runExploreAreaThread, 5, TimeUnit.MILLISECONDS);
   }

   private boolean readyToTransitionFromStopToLookAround(double timeInState)
   {
      return explore.get();
   }

   private boolean readyToTransitionFromLookAroundToPerceive(double timeInState)
   {
      return (timeInState > turnChestTrajectoryDuration);
   }

   private boolean readyToTransitionFromPerceiveToGrabPlanarRegions(double timeInState)
   {
      return (timeInState > perceiveDuration);
   }

   private boolean readyToTransitionFromGrabPlanarRegionsToLookAround(double timeInState)
   {
      return (timeInState > 1.0);
   }

   private boolean readyToTransitionFromGrabPlanarRegionsToStop(double timeInState)
   {
      return (chestYawForLookingAroundIndex >= chestYawsForLookingAround.size());
   }

   private void runExploreAreaThread()
   {
      stateMachine.doActionAndTransition();
   }

   private void onStopStateEntry()
   {
      LogTools.info("onStopStateEntry");
      chestYawForLookingAroundIndex = 0;
      behaviorHelper.pauseWalking();
   }

   private void doStopStateAction(double timeInState)
   {
   }

   private void onLookAroundStateEntry()
   {
      LogTools.info("onLookAroundStateEntry");

      double chestYaw = Math.toRadians(chestYawsForLookingAround.get(chestYawForLookingAroundIndex++));
      turnChest(chestYaw, turnChestTrajectoryDuration);
   }

   private void doLookAroundStateAction(double timeInState)
   {
   }

   private void onPerceiveStateEntry()
   {
      LogTools.info("onPerceiveStateEntry");
      behaviorHelper.clearREA();
   }

   private void rememberObservationPoint()
   {
      //TODO: Remember the LIDAR pointing at transform instead of just where the robot was at. But how to get that frame?
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint3D midFeetLocation = new FramePoint3D(midFeetZUpFrame);
      midFeetLocation.changeFrame(worldFrame);
      
      this.pointsObservedFrom.add(new Point3D(midFeetLocation));
   }

   private void doPerceiveStateAction(double timeInState)
   {
   }

   private void onGrabPlanarRegionsStateEntry()
   {
      LogTools.info("onGrabPlanarRegionsStateEntry");

      rememberObservationPoint();
      PlanarRegionsList latestPlanarRegionsList = behaviorHelper.getLatestPlanarRegionList();

      if (concatenatedMap == null)
      {
         concatenatedMap = latestPlanarRegionsList;
      }
      else
      {
         // Add the newest Planar regions to the map.

         List<PlanarRegion> planarRegionsAsList = latestPlanarRegionsList.getPlanarRegionsAsList();
         
         for (PlanarRegion planarRegion : planarRegionsAsList)
         {
            concatenatedMap.addPlanarRegion(planarRegion);
         }
         
         LogTools.info("concatenatedMap has " + concatenatedMap.getNumberOfPlanarRegions() + " planar Regions");

         // Send it to the GUI for a viz...
//         PlanarRegionsListMessage concatenatedMapMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(concatenatedMap);
//         messager.submitMessage(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ConcatenatedMap, concatenatedMapMessage);
         
         
         // Find a point that has not been observed, but is close to a point that can be walked to, in order to observe it...
         
         
      }
      
      
      
   }

   private void doGrabPlanarRegionsStateAction(double timeInState)
   {
   }

   public void turnChest(double chestYaw, double trajectoryTime)
   {
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FrameQuaternion chestOrientation = new FrameQuaternion(midFeetZUpFrame, chestYaw, 0.0, 0.0);
      chestOrientation.changeFrame(worldFrame);
      behaviorHelper.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, referenceFrames.getPelvisZUpFrame());

      behaviorHelper.requestPelvisGoHome(trajectoryTime);
   }

   public static class ExploreAreaBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("ExploreAreaBehavior");
      private static final CategoryTheme ExploreAreaTheme = apiFactory.createCategoryTheme("ExploreArea");
      private static final Category ExploreAreaCategory = RootCategory.child(ExploreAreaTheme);

      public static final Topic<Boolean> ExploreArea = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("ExploreArea"));
      public static final Topic<PlanarRegionsListMessage> ConcatenatedMap = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("ConcatenatedMap"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }

}
