package us.ihmc.moonwalking.models.InvertedPendulumWithFoot;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class InvertedPendulumWithFootSimulation
{
  private static final boolean BENCHMARK = false;

  public static final double DT = 0.0002;
  public InvertedPendulumWithFootSimulation()
  {
    double pendulumMass = 100.0;
    double pendulumLength = 1.0;
    double gravity = 9.81/6.0;

    InvertedPendulumWithFoot invertedPendulumWithFoot = new InvertedPendulumWithFoot(gravity, pendulumLength, pendulumMass);
    InvertedPendulumWithFootController invertedPendulumWithFootController = new InvertedPendulumWithFootController(invertedPendulumWithFoot);

    invertedPendulumWithFoot.setController(invertedPendulumWithFootController);

    double initialAngle = 0.0;
    double initialAnglularVelocity = 0.0;
    invertedPendulumWithFoot.setInitialAngleAndVelocity(initialAngle, initialAnglularVelocity);

//    DEFAULT_NOMLEN = 0.008, DEFAULT_K_XY = 1422, DEFAULT_B_XY = 15.6, DEFAULT_K_Z = 125, DEFAULT_B_Z = 300;
    LinearGroundContactModel linearGroundContactModel = new LinearGroundContactModel(invertedPendulumWithFoot, 15000.0, 2000.0, 1000.0, 3000.0);
    invertedPendulumWithFoot.setGroundContactModel(linearGroundContactModel);


//    startGainTuning(invertedPendulumWithFootController, invertedPendulumWithFoot);
    startRegionOfConvergenceDetermining(invertedPendulumWithFootController, invertedPendulumWithFoot);
//    startSimulationGUI(invertedPendulumWithFootController, invertedPendulumWithFoot);
  }

  private void startSimulationGUI(InvertedPendulumWithFootController invertedPendulumWithFootController, InvertedPendulumWithFoot invertedPendulumWithFoot)
  {
    double initialAngle = -0.5;
    double initialAnglularVelocity = 0.5;
    invertedPendulumWithFoot.setInitialAngleAndVelocity(initialAngle, initialAnglularVelocity);

    invertedPendulumWithFootController.setKp(0.0);
    invertedPendulumWithFootController.setKd(0.0);

    SimulationConstructionSet scs = new SimulationConstructionSet(invertedPendulumWithFoot);

    setUpGUI(scs);
//    scs.selectConfiguration("Control");
    scs.selectConfiguration("InitalStudy");


    ArrayList<DynamicGraphicObjectsList> list = invertedPendulumWithFoot.getDynamicGraphicObjectsList();

    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsRegistry = new DynamicGraphicObjectsListRegistry();
    dynamicGraphicObjectsRegistry.registerDynamicGraphicObjectsLists(list);

    dynamicGraphicObjectsRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

//    scs.setGroundVisible(false);

    scs.setDT(DT, 200);

    Thread thread = new Thread(scs);
    thread.setName("SimulationConstructionSet");
    thread.start();

    if (BENCHMARK)
    {
      scs.simulate();

      while (true)
      {
        try
        {
          Thread.sleep(1000);
        }
        catch (InterruptedException ex)
        {
        }

        double time = invertedPendulumWithFoot.getTime();

        System.out.println("t = " + time);
      }
    }
  }

  private void startGainTuning(InvertedPendulumWithFootController invertedPendulumWithFootController,
                               InvertedPendulumWithFoot invertedPendulumWithFoot)
  {
    SimulationConstructionSet scs = new SimulationConstructionSet(invertedPendulumWithFoot, true);

    double startingKp = 1.0;
    double endingKp = 1.2;
    double deltaKp = (endingKp - startingKp) / 10.0;

    double startingKd = 0.15;
    double endingKd = 0.25;
    double deltaKd = (endingKd - startingKd) / 10.0;

    SimulateOutOfControl simulateOutOfControl = new SimulateOutOfControl(invertedPendulumWithFootController);
    SimilationDone similationDone = new SimilationDone();

    scs.setSimulateDoneCriterion(simulateOutOfControl);
    scs.addSimulateDoneListener(similationDone);

    scs.setDT(DT, 200);

    setUpGUI(scs);

    scs.selectConfiguration("Control");

    double maximumSimulationTime = 12.0;

    double initialAngle = 0.1;

    Thread thread = new Thread(scs);
    thread.setName("SimulationConstructionSet");
    thread.start();

//    double kp = 1.1;
//    double kd = 0.2;

    for (double kp = startingKp; kp <= endingKp; kp += deltaKp)
    {
      for (double kd = startingKd; kd <= endingKd; kd += deltaKd)
      {
        similationDone.reset();
        System.out.println("\n\n**********");
        System.out.println("starting sim for:" + " kp=" + kp + ", kd=" + kd);
        invertedPendulumWithFoot.setInitialAngle(initialAngle);
        invertedPendulumWithFoot.setTime(0.0);
        invertedPendulumWithFootController.reset();

        invertedPendulumWithFootController.setKp(kp);
        invertedPendulumWithFootController.setKd(kd);

        scs.simulate(maximumSimulationTime);


        scs.setDT(DT, 999999999);


        while (!similationDone.isSimulationDone())
        {

          try
          {
            Thread.sleep(1000);
          }
          catch (InterruptedException ex)
          {
          }

//          System.out.println("time= " + invertedPendulumWithFoot.getTime());
        }

        scs.tickAndUpdate();

        //check how the simulation finished:
        double maximumPercentOvershoot = invertedPendulumWithFootController.getPercentOvershoot();
        double simulationTime = invertedPendulumWithFoot.getTime();

        if (simulationTime < 0.99 * maximumSimulationTime)
        {
          //the simulation was aborted
          System.out.println("ABORTED: overshoot=" + maximumPercentOvershoot + ", kp=" + kp + ", kd=" + kd);
        }
        else
        {
          System.out.println("GOOD   : overshoot=" + maximumPercentOvershoot + ", kp=" + kp + ", kd=" + kd);
        }

      }
    }

    System.out.println("\nDone");
  }


  private void startRegionOfConvergenceDetermining(InvertedPendulumWithFootController invertedPendulumWithFootController,
                              InvertedPendulumWithFoot invertedPendulumWithFoot)
 {

//   invertedPendulumWithFootController.setKp(0.0);
//   invertedPendulumWithFootController.setKd(0.0);

   SimulationConstructionSet scs = new SimulationConstructionSet(invertedPendulumWithFoot, true);


   YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("TrialVariables");

   YoVariable initalAngle = new YoVariable("initalPosition", yoVariableRegistry);
   YoVariable initalAngularSpeed = new YoVariable("initalAngularSpeed", yoVariableRegistry);

//   YoVariable axisAngle = new YoVariable("axisAngle", yoVariableRegistry);
//   YoVariable axisSpeed = new YoVariable("axisSpeed", yoVariableRegistry);

   VarList varList = yoVariableRegistry.createVarList();
   scs.addVarList(varList);

//   scs.re

   double numberOfSteps = 15.0;
   double startingAngle = -0.7;
   double endingAngle = 0.7;
   double deltaAngle = (endingAngle - startingAngle) / numberOfSteps + 1e-6;

   double startingAngularSpeed = -0.7;
   double endingAngularSpeed = 0.7;
   double deltaAngularSpeed = (endingAngularSpeed - startingAngularSpeed) / numberOfSteps + 1e-6;

   SimulateOutOfControl simulateOutOfControl = new SimulateOutOfControl(invertedPendulumWithFootController);
   SimilationDone similationDone = new SimilationDone();

   scs.setSimulateDoneCriterion(simulateOutOfControl);
   scs.addSimulateDoneListener(similationDone);

   scs.setDT(DT, 200);


   setUpGUI(scs);
   scs.selectConfiguration("PhasePortrait");
   scs.changeBufferSize(500000);

   double maximumSimulationTime = 40.0;

   Thread thread = new Thread(scs);
   thread.setName("SimulationConstructionSet");
   thread.start();

//    double kp = 1.1;
//    double kd = 0.2;

   scs.setDT(DT, 100);

//   createAxesForPhasePlot(scs, invertedPendulumWithFoot, 0.0, 0.0);


   for (initalAngle.val = startingAngle; initalAngle.val <= endingAngle; initalAngle.val += deltaAngle)
   {
     for (initalAngularSpeed.val = startingAngularSpeed; initalAngularSpeed.val <= endingAngularSpeed; initalAngularSpeed.val += deltaAngularSpeed)
     {
       similationDone.reset();
       System.out.println("\n\n**********");
       System.out.println("starting sim for:" + " angle=" + initalAngle.val + ", anglularSpeed=" + initalAngularSpeed.val);
       invertedPendulumWithFoot.setInitialAngleAndVelocity(initalAngle.val, initalAngularSpeed.val);
       invertedPendulumWithFoot.setTime(0.0);
       invertedPendulumWithFootController.reset();

//       invertedPendulumWithFootController.setKp(kp);
//       invertedPendulumWithFootController.setKd(kd);

       scs.simulate(maximumSimulationTime);





       while (!similationDone.isSimulationDone())
       {

         try
         {
           Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }

//          System.out.println("time= " + invertedPendulumWithFoot.getTime());
       }

       scs.tickAndUpdate();

       //check how the simulation finished:
       double maximumPercentOvershoot = invertedPendulumWithFootController.getPercentOvershoot();
       double simulationTime = invertedPendulumWithFoot.getTime();

//       if (simulationTime < 0.99 * maximumSimulationTime)
//       {
//         //the simulation was aborted
//         System.out.println("ABORTED: overshoot=" + maximumPercentOvershoot + ", kp=" + kp + ", kd=" + kd);
//       }
//       else
//       {
//         System.out.println("GOOD   : overshoot=" + maximumPercentOvershoot + ", kp=" + kp + ", kd=" + kd);
//       }

     }
   }

   System.out.println("\nDone");
 }

  /**
   * createAxesForPhasePlot
   *
   * @param scs SimulationConstructionSet
   * @param axisAngle YoVariable
   * @param axisSpeed YoVariable
   */
  private void createAxesForPhasePlot(SimulationConstructionSet scs, InvertedPendulumWithFoot invertedPendulumWithFoot, double initialAngle, double initialSpeed)
  {
    double maxAngle = 0.1;
    double maxspeed = 0.1;

    invertedPendulumWithFoot.setInitialAngleAndVelocity(-maxAngle, 0.0);
    scs.tickAndUpdate();

    invertedPendulumWithFoot.setInitialAngleAndVelocity(maxAngle, 0.0);
    scs.tickAndUpdate();

    invertedPendulumWithFoot.setInitialAngleAndVelocity(0.0, 0.0);
    scs.tickAndUpdate();

    invertedPendulumWithFoot.setInitialAngleAndVelocity(0.0, maxspeed);
    scs.tickAndUpdate();

    invertedPendulumWithFoot.setInitialAngleAndVelocity(0.0, -maxspeed);
    scs.tickAndUpdate();

    invertedPendulumWithFoot.setInitialAngleAndVelocity(0.0, 0.0);
    scs.tickAndUpdate();

    invertedPendulumWithFoot.setInitialAngleAndVelocity(initialAngle, initialSpeed);
    scs.tickAndUpdate();
  }

  private class SimulateOutOfControl implements SimulateDoneCriterion
  {
    private final InvertedPendulumWithFootController invertedPendulumWithFootController;

    private SimulateOutOfControl(InvertedPendulumWithFootController invertedPendulumWithFootController)
    {
      this.invertedPendulumWithFootController = invertedPendulumWithFootController;
    }

    public boolean isSimulationDone()
    {
      return invertedPendulumWithFootController.doesControllerGiveUpOrSettled();
    }
  }

  private class SimilationDone implements SimulateDoneListener
  {
    boolean simulationIsDone;
    private SimilationDone()
    {
      reset();
    }

    public void simulateDone()
    {
      simulationIsDone = true;
    }

    public boolean isSimulationDone()
    {
      return simulationIsDone;
    }

    public void reset()
    {
      simulationIsDone = false;
    }

  }

  public static void main(String[] args)
  {
    InvertedPendulumWithFootSimulation invertedPendulumWithFootSimulation = new InvertedPendulumWithFootSimulation();
  }

  private void setUpGUI(SimulationConstructionSet scs)
  {
    CameraConfiguration view1 = new CameraConfiguration("view 1");
    view1.setCameraDolly(false, true, true, false);
    view1.setCameraTracking(false, true, true, false);
    view1.setCameraPosition(0.78, -1.7, 0.6);
    view1.setCameraFix(0.4, -0.2, 0.15);
    scs.setupCamera(view1);

    scs.setupGraphGroup("Control", new String[][][]
                        {
                        {
                          {"error"}, {"auto"}
                        },
//                            {
//                            {"error_d"}, {"auto"}
//                        },

                            {
                        {"tau"},
                        {"auto"}
    },
                        {{"percentOvershoot", "percentOvershoot_calc", "riseTime"},{""}},
                        {{"omegaN_calc", "omegaD_calc", "omegaD"},{""}},
                        {{"zeta"},{""}},
                        {{"settled"},{""}},
                        {{"qd_pitch", "bodyPitchDotFilt"},{""}},


    }, 1);

    GraphConfiguration phasePlot = new GraphConfiguration("phasePlot", GraphConfiguration.AUTO_SCALING);
//    System.out.println("before plot type= " + phasePlot.getPlotType());
    phasePlot.setPlotType(GraphConfiguration.PHASE_PLOT);
//    System.out.println("after plot type= " + phasePlot.getPlotType());
    scs.setupGraphConfigurations(new GraphConfiguration[]{phasePlot});


    scs.setupGraphGroup("PhasePortrait", new String[][][]
                        {
//                        {{"q_pitch", "bodyPitchDotFilt"},{"phasePlot"}},
                        {{"q_pitch", "qd_pitch"},{"phasePlot"}},
    }, 1);

    scs.setupGraphGroup("InitalStudy", new String[][][]
                       {
//                        {{"q_pitch", "bodyPitchDotFilt"},{"phasePlot"}},
                       {{"error"},{"auto"}},
                       {{"q_pitch"},{"auto"}},
                       {{"qd_pitch"},{"auto"}},
                       {{"q_ankle"},{"auto"}},
                       {{"qd_ankle"},{"auto"}},
   }, 1);

    scs.setupConfiguration("InitalStudy", "all", "InitalStudy", "Control");

    scs.setupVarGroup("PhasePortrait", new String[]
           {"t", "q_pitch", "qd_pitch", "settled"});

    scs.setupEntryBoxGroup("Control", new String[]
                           {
                           "kp",
                           "kd",
    });

    scs.setupConfiguration("PhasePortrait", "PhasePortrait", "PhasePortrait", "Control");

    scs.setupConfiguration("Control", "all", "Control", "Control");


  }
}
