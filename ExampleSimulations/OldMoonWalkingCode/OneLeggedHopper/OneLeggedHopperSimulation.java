package us.ihmc.moonwalking.models.OneLeggedHopper;

import java.util.ArrayList;

import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

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
public class OneLeggedHopperSimulation
{
  public static final double DT = 0.0002;

  public OneLeggedHopperSimulation()
  {
    double gravity = 9.81 / 6.0;

    OneLeggedHopperRobot oneLeggedHopperRobot = new OneLeggedHopperRobot(gravity);

    OneLeggedHopperController oneLeggedHopperController = new OneLeggedHopperController(oneLeggedHopperRobot);

    oneLeggedHopperRobot.setController(oneLeggedHopperController);



//    DEFAULT_NOMLEN = 0.008, DEFAULT_K_XY = 1422, DEFAULT_B_XY = 15.6, DEFAULT_K_Z = 125, DEFAULT_B_Z = 300;
    LinearGroundContactModel linearGroundContactModel = new LinearGroundContactModel(oneLeggedHopperRobot, 15000.0, 2000.0, 1000.0, 3000.0);
    oneLeggedHopperRobot.setGroundContactModel(linearGroundContactModel);

    startSimulationGUI(oneLeggedHopperRobot);
  }

  private void startSimulationGUI(OneLeggedHopperRobot oneLeggedHopperRobot)
  {
    double initialAngle = -0.5;
    double initialAnglularVelocity = 0.5;
//    oneLeggedHopperRobot.setInitialAngleAndVelocity(initialAngle, initialAnglularVelocity);

    SimulationConstructionSet scs = new SimulationConstructionSet(oneLeggedHopperRobot);

    setUpGUI(scs);
//    scs.selectConfiguration("Control");
    scs.selectConfiguration("Control");

    ArrayList<DynamicGraphicObjectsList> list = oneLeggedHopperRobot.getDynamicGraphicObjectsList();

    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsRegistry = new DynamicGraphicObjectsListRegistry();
    dynamicGraphicObjectsRegistry.registerDynamicGraphicObjectsLists(list);

    dynamicGraphicObjectsRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

//    scs.setGroundVisible(false);

    scs.setDT(DT, 200);

    Thread thread = new Thread(scs);
    thread.setName("SimulationConstructionSet");
    thread.start();

//    if (BENCHMARK)
//    {
//      scs.simulate();
//
//      while (true)
//      {
//        try
//        {
//          Thread.sleep(1000);
//        }
//        catch (InterruptedException ex)
//        {
//        }
//
//        double time = invertedPendulumWithFoot.getTime();
//
//        System.out.println("t = " + time);
//      }
//    }

  }

  private void setUpGUI(SimulationConstructionSet scs)
{
  CameraConfiguration view1 = new CameraConfiguration("view1");
  view1.setCameraDolly(false, true, true, false);
  view1.setCameraTracking(false, true, true, false);
  view1.setCameraPosition(0.0, -9.6, 1.2);
  view1.setCameraFix(-0.29, 0.065, 0.88);
  scs.setupCamera(view1);

  scs.selectCamera("view1");

  scs.setupGraphGroup("Control", new String[][][]
                      {
                      {{"t"}, {"auto"}},
                      {{"q_legJoint", "qLeg_d"}, {"auto"}},
                      {{"qd_legJoint", "qdLeg_d"}, {"auto"}},
                      {{"errorLeg"}, {"auto"}},
                      {{"errorLegD",}, {"auto"}},
                      {{"tauLeg"}, {"auto"}},
                      {{"state"}, {""}},
  }, 1);

  GraphConfiguration phasePlot = new GraphConfiguration("phasePlot", GraphConfiguration.AUTO_SCALING);
  phasePlot.setPlotType(GraphConfiguration.PHASE_PLOT);
  scs.setupGraphConfigurations(new GraphConfiguration[]{phasePlot});

  scs.setupGraphGroup("PhasePortrait", new String[][][]
                      {
                      {{"q_pitch", "qd_pitch"},{"phasePlot"}},
  }, 1);

  scs.setupGraphGroup("InitalStudy", new String[][][]
                     {
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
                         "kpLeg","kdLeg",
  });

  scs.setupConfiguration("PhasePortrait", "PhasePortrait", "PhasePortrait", "Control");

  scs.setupConfiguration("Control", "all", "Control", "Control");
}


  public static void main(String[] args)
  {
    OneLeggedHopperSimulation oneLeggedHopperSimulation = new OneLeggedHopperSimulation();
  }
}
