package us.ihmc.commonWalkingControlModules.trajectories;


import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiSliderBoard;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.CartesianTrajectoryGenerator;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class CartesianTrajectoryGeneratorTester
{
   private static final double DT = 0.005;
   private String name;

   public CartesianTrajectoryGeneratorTester(CartesianTrajectoryGenerator cartesianTrajectoryGenerator, YoVariableRegistry registry,
           YoGraphicsListRegistry yoGraphicsListRegistry, String name)
   {
      this.name = name;
      Robot nullRobot = new Robot("null")
      {
         private static final long serialVersionUID = 629274113314836560L;
      };

      CartesianTrajectoryGeneratorTesterController controller = new CartesianTrajectoryGeneratorTesterController((DoubleYoVariable) nullRobot.getVariable("t"),
                                                                   cartesianTrajectoryGenerator, yoGraphicsListRegistry,
                                                                   "cartesianTrajectoryGeneratorTesterController");
      nullRobot.setController(controller);

      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot);
      scs.setDT(DT, 1);

      scs.addVarLists(registry.createVarListsIncludingChildren());

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

//    yoVariableRegistry.addVarListsToSimulationConstructionSet(scs);


      scs.setupGraphGroup("CartesianTrajectory", new String[][][]
      {
         {
            {"currentPositionx", "finalDesiredx"}, {"auto"}
         },
         {
            {"currentPositiony", "finalDesiredy"}, {"auto"}
         },
         {
            {"currentPositionz", "finalDesiredz"}, {"auto"}
         },
         {
            {"currentVelocityx"}, {"auto"}
         },
         {
            {"currentVelocityy"}, {"auto"}
         },
         {
            {"currentVelocityz"}, {"auto"}
         },
         {
            {"currentAccelerationx"}, {"auto"}
         },
         {
            {"currentAccelerationy"}, {"auto"}
         },
         {
            {"currentAccelerationz"}, {"auto"}
         },
         {
            {"currentDistanceFromTarget"}, {"auto"}
         },
         {
            {"currentVelocityMag", "maxVel"}, {"auto"}
         },
         {
            {"currentAccelMag", "maxAccel"}, {"auto"}
         },
         {
            {"accelFull"}, {"auto"}
         },
      }, 3);

      scs.setupEntryBoxGroup("CartesianTrajectory", new String[]
      {
         "maxVel", "maxAccel", "zClearance", "landingDistance", "finalDesiredx", "finalDesiredy", "finalDesiredz", "reset"
      });

      scs.setupConfiguration("CartesianTrajectory", "all", "CartesianTrajectory", "CartesianTrajectory");

      scs.selectConfiguration("CartesianTrajectory");

      MidiSliderBoard evolution = new MidiSliderBoard(scs);
      evolution.setSlider(1, "endPointShiftx", scs, -0.5, 0.5);
      evolution.setSlider(2, "endPointShifty", scs, -0.5, 0.5);
      evolution.setSlider(3, "allowEndPointShift", scs, 0.0, 1.0);
      evolution.setSlider(4, "slowDownMillis", scs, 0.0, 100);

      Thread thread = new Thread(scs);
      thread.start();
   }

   private static final double[][] generateStandardFinalPoints()
   {
      double[][] ret = new double[][]
      {
         {0.01, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.25, 0.0, 0.0}, {0.0, -0.5, 0.0}, {0.5, 0.0, 0.0}, {0.75, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.5, 0.0},
      };

      return ret;
   }


   private class CartesianTrajectoryGeneratorTesterController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("CartesianTrajectoryGeneratorTesterController");
      private final CartesianTrajectoryGenerator cartesianTrajectoryGenerator;
      private final BagOfBalls bagOfBalls;

      private final FramePoint startingTestPoint;
      private final FrameVector startingTestVelocity;
      private final ArrayList<FramePoint> testPointsToCycleThrough = new ArrayList<FramePoint>();

      private final DoubleYoVariable t;

      private final DoubleYoVariable resetEvery = new DoubleYoVariable("resetEvery", registry);
      private final DoubleYoVariable lastResetTime = new DoubleYoVariable("lastResetTime", registry);
      private final IntegerYoVariable testPointIndex = new IntegerYoVariable("testPointIndex", registry);

      private final BooleanYoVariable allowEndPointShift = new BooleanYoVariable("allowEndPointShift", registry);
      private final IntegerYoVariable slowDownMillis = new IntegerYoVariable("slowDownMillis", registry);


      private final BooleanYoVariable reset = new BooleanYoVariable("reset", registry);
      private final YoFramePoint originalFinalDesiredPosition = new YoFramePoint("originalFinalDesiredPosition", "", ReferenceFrame.getWorldFrame(), registry);
      private final YoFramePoint finalDesiredPosition = new YoFramePoint("finalDesired", "", ReferenceFrame.getWorldFrame(), registry);

      private final YoFramePoint currentPosition = new YoFramePoint("currentPosition", "", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector currentVelocity = new YoFrameVector("currentVelocity", "", ReferenceFrame.getWorldFrame(), registry);
      private final YoFrameVector currentAcceleration = new YoFrameVector("currentAcceleration", "", ReferenceFrame.getWorldFrame(), registry);

      private final YoFrameVector endPointShift = new YoFrameVector("endPointShift", "", ReferenceFrame.getWorldFrame(), registry);
      private String name;



      public CartesianTrajectoryGeneratorTesterController(DoubleYoVariable t, CartesianTrajectoryGenerator cartesianTrajectoryGenerator,
              YoGraphicsListRegistry yoGraphicsListRegistry, String name)
      {
         this(t, cartesianTrajectoryGenerator, new double[] {0.0, 0.0, 0.0}, generateStandardFinalPoints(), yoGraphicsListRegistry, name);
      }

      public CartesianTrajectoryGeneratorTesterController(DoubleYoVariable t, CartesianTrajectoryGenerator cartesianTrajectoryGenerator,
              double[] startingTestPoint, double[][] testPointsToCycleThrough, YoGraphicsListRegistry yoGraphicsListRegistry, String name)
      {
         this.name = name;
         this.t = t;

         allowEndPointShift.set(true);
         slowDownMillis.set(0);

         resetEvery.set(4.0);    // 4.0;

         this.startingTestPoint = new FramePoint(ReferenceFrame.getWorldFrame(), startingTestPoint);
         startingTestVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

         for (double[] testPoint : testPointsToCycleThrough)
         {
            this.testPointsToCycleThrough.add(new FramePoint(ReferenceFrame.getWorldFrame(), testPoint));
         }

         this.cartesianTrajectoryGenerator = cartesianTrajectoryGenerator;
         originalFinalDesiredPosition.set(this.testPointsToCycleThrough.get(0));
         finalDesiredPosition.set(originalFinalDesiredPosition);
         finalDesiredPosition.add(endPointShift);

         cartesianTrajectoryGenerator.initialize(this.startingTestPoint, startingTestVelocity, null, this.testPointsToCycleThrough.get(0), null);

         bagOfBalls = BagOfBalls.createPatrioticBag(500, 0.006, "tester", registry, yoGraphicsListRegistry);

//       finalDesiredPosition.set(0.0, 2.0, 1.0);
//       reset.set(true);

         if (yoGraphicsListRegistry != null)
         {
            YoGraphicsList yoGraphicsList = new YoGraphicsList("CartesianTrajectoryTester");

            yoGraphicsList.add(new YoGraphicPosition("Final Desired", finalDesiredPosition, 0.02, YoAppearance.Red()));
            yoGraphicsList.add(new YoGraphicPosition("Original Final Desired", originalFinalDesiredPosition, 0.02, YoAppearance.Black()));
            yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         }
      }

      private final FramePoint currentPositionFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      private final FrameVector currentVelocityFramePoint = new FrameVector(ReferenceFrame.getWorldFrame());
      private final FrameVector currentAccelerationFramePoint = new FrameVector(ReferenceFrame.getWorldFrame());

      private int skipCount = 0;
      private int showBallEveryN = 5;

      public void doControl()
      {
         try
         {
            Thread.sleep(slowDownMillis.getIntegerValue());
         }
         catch (InterruptedException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         if (t.getDoubleValue() - lastResetTime.getDoubleValue() > resetEvery.getDoubleValue())
         {
            lastResetTime.set(t.getDoubleValue());

            reset.set(true);
            testPointIndex.set(testPointIndex.getIntegerValue() + 1);

            originalFinalDesiredPosition.set(testPointsToCycleThrough.get(testPointIndex.getIntegerValue() % testPointsToCycleThrough.size()));
            finalDesiredPosition.set(originalFinalDesiredPosition);

            cartesianTrajectoryGenerator.initialize(startingTestPoint, startingTestVelocity, null, finalDesiredPosition.getFramePointCopy(), null);

//          cartesianTrajectoryGenerator.updateFinalDesiredPosition(finalDesiredPosition.getFramePointCopy());

//          bagOfBalls.reset();
         }

         if (allowEndPointShift.getBooleanValue())
         {
            finalDesiredPosition.set(originalFinalDesiredPosition);
            finalDesiredPosition.add(endPointShift);
            cartesianTrajectoryGenerator.updateFinalDesiredPosition(finalDesiredPosition.getFramePointCopy());
         }


         if (reset.getBooleanValue())
         {
            cartesianTrajectoryGenerator.updateFinalDesiredPosition(finalDesiredPosition.getFramePointCopy());
            reset.set(false);
            bagOfBalls.reset();
         }
         else
         {
            cartesianTrajectoryGenerator.computeNextTick(currentPositionFramePoint, currentVelocityFramePoint, currentAccelerationFramePoint, DT);

            skipCount++;

            if (skipCount > showBallEveryN)
            {
               bagOfBalls.setBall(currentPositionFramePoint);
               skipCount = 0;
            }

            currentPosition.set(currentPositionFramePoint);
            currentVelocity.set(currentVelocityFramePoint);
            currentAcceleration.set(currentAccelerationFramePoint);
         }
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return name;
      }
      
      public void initialize()
      {      
      }

      public String getDescription()
      {
         return getName();
      }

   }
}
