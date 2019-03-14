package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WrapperForMultipleToeOffCalculatorsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double footLengthForControl = 0.22;
   private static final double footWidthForControl = 0.11;
   private static final double toeWidthForControl = 0.0825;

   private WrapperForMultipleToeOffCalculators generator;
   private EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators;
   private YoVariableRegistry parentRegistry;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   @BeforeEach
   public void setUp()
   {
      parentRegistry = new YoVariableRegistry("parentRegistryTEST");
      toeOffCalculators = new EnumMap<>(ToeOffEnum.class);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = new FramePose3D();
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.20));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, parentRegistry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ToeOffCalculator toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates, contactableFeet, getWalkingControllerParameters().getToeOffParameters(), parentRegistry);
      toeOffCalculators.put(toeOffCalculator.getEnum(), toeOffCalculator);
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }



	@Test
   public void testConstructor()
   {
      generator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, parentRegistry);
   }


   @Test
   public void testClear()
   {
      generator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, parentRegistry);
      generator.clear();
   }

   @Test
   public void testSetExitCMP()
   {
      RobotSide trailingSide = RobotSide.LEFT;
      FramePoint3D exitCMP = new FramePoint3D();
      exitCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      exitCMP.setX(0.05);

      generator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, parentRegistry);
      generator.setExitCMP(exitCMP, trailingSide);
   }


   @Test
   public void testComputeToeOffContactPoint()
   {
      RobotSide trailingSide = RobotSide.LEFT;

      FramePoint3D exitCMP = new FramePoint3D();
      FramePoint2D desiredCMP = new FramePoint2D();

      exitCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      desiredCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());

      exitCMP.setX(0.05);
      desiredCMP.setX(0.05);

      generator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, parentRegistry);
      generator.setExitCMP(exitCMP, trailingSide);
      generator.computeToeOffContactPoint(desiredCMP, trailingSide);
   }

   @Test
   public void testGetToeOffContactPoint()
   {
      RobotSide trailingSide = RobotSide.LEFT;

      FramePoint3D exitCMP = new FramePoint3D();
      FramePoint2D desiredCMP = new FramePoint2D();
      FramePoint2D toeOffPoint = new FramePoint2D();

      exitCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      desiredCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      toeOffPoint.setToZero(contactableFeet.get(trailingSide).getSoleFrame());

      exitCMP.setX(0.05);
      desiredCMP.setX(0.05);

      generator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, parentRegistry);
      generator.setExitCMP(exitCMP, trailingSide);
      generator.computeToeOffContactPoint(desiredCMP, trailingSide);
      generator.getToeOffContactPoint(toeOffPoint, trailingSide);
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return new WalkingControllerParameters()
      {
         @Override
         public double getOmega0()
         {
            return 0;
         }

         @Override
         public double getMaximumLegLengthForSingularityAvoidance()
         {
            return 0;
         }

         @Override
         public double minimumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double nominalHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double maximumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double defaultOffsetHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
         {
            return false;
         }

         @Override
         public boolean allowAutomaticManipulationAbort()
         {
            return false;
         }

         @Override
         public double getMinimumSwingTimeForDisturbanceRecovery()
         {
            return 0;
         }

         @Override
         public double getICPErrorThresholdToSpeedUpSwing()
         {
            return 0;
         }

         @Override
         public ICPControlGains createICPControlGains()
         {
            return null;
         }

         @Override
         public PDGains getCoMHeightControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getSwingFootControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getHoldPositionFootControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getToeOffFootControlGains()
         {
            return null;
         }

         @Override
         public double getDefaultTransferTime()
         {
            return 0;
         }

         @Override
         public double getDefaultSwingTime()
         {
            return 0;
         }

         @Override
         public FootSwitchFactory getFootSwitchFactory()
         {
            return null;
         }

         @Override
         public String[] getJointsToIgnoreInController()
         {
            return new String[0];
         }

         @Override
         public MomentumOptimizationSettings getMomentumOptimizationSettings()
         {
            return null;
         }

         @Override
         public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
         {
            return null;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportX()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportY()
         {
            return 0;
         }

         @Override
         public boolean finishSingleSupportWhenICPPlannerIsDone()
         {
            return false;
         }

         @Override
         public double getHighCoPDampingDurationToPreventFootShakies()
         {
            return 0;
         }

         @Override
         public double getCoPErrorThresholdForHighCoPDamping()
         {
            return 0;
         }

         @Override
         public ToeOffParameters getToeOffParameters()
         {
            return new ToeOffParameters()
            {
               @Override
               public boolean doToeOffIfPossible()
               {
                  return false;
               }

               @Override
               public boolean doToeOffIfPossibleInSingleSupport()
               {
                  return false;
               }

               @Override
               public boolean checkECMPLocationToTriggerToeOff()
               {
                  return false;
               }

               @Override
               public double getMinStepLengthForToeOff()
               {
                  return 0;
               }

               @Override
               public boolean doToeOffWhenHittingAnkleLimit()
               {
                  return false;
               }

               @Override
               public double getMaximumToeOffAngle()
               {
                  return 0;
               }
            };
         }

         @Override
         public SwingTrajectoryParameters getSwingTrajectoryParameters()
         {
            return new SwingTrajectoryParameters()
            {
               @Override
               public boolean doToeTouchdownIfPossible()
               {
                  return false;
               }

               @Override
               public double getToeTouchdownAngle()
               {
                  return 0;
               }

               @Override
               public boolean doHeelTouchdownIfPossible()
               {
                  return false;
               }

               @Override
               public double getHeelTouchdownAngle()
               {
                  return 0;
               }

               @Override
               public double getDesiredTouchdownHeightOffset()
               {
                  return 0;
               }

               @Override
               public double getDesiredTouchdownVelocity()
               {
                  return 0;
               }

               @Override
               public double getDesiredTouchdownAcceleration()
               {
                  return 0;
               }

               @Override
               public double getMinMechanicalLegLength()
               {
                  return 0;
               }
            };
         }

         @Override
         public ICPOptimizationParameters getICPOptimizationParameters()
         {
            return null;
         }

         @Override
         public SteppingParameters getSteppingParameters()
         {
            return null;
         }
      };
   }
}
