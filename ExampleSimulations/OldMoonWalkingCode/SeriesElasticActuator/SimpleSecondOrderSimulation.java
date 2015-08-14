package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.CameraConfiguration;

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
public class SimpleSecondOrderSimulation
{
  public static final double DT = 0.000001;

  public SimpleSecondOrderSimulation()
  {
    SimpleSecondOrderSystemModel simpleSecondOrderSystemModel = new SimpleSecondOrderSystemModel();

    SimpleSecondOrderSystemController simpleSecondOrderSystemController = new SimpleSecondOrderSystemController(simpleSecondOrderSystemModel);

    double controlDT = DT; //0.00001; //0.001;
    int simulationTicksPerControlTick = (int) (Math.round(controlDT / DT));
    simpleSecondOrderSystemModel.setController(simpleSecondOrderSystemController, simulationTicksPerControlTick);

    SimulationConstructionSet scs = new SimulationConstructionSet(simpleSecondOrderSystemModel);

    scs.changeBufferSize(64000);
    setUpGUI(scs);
    scs.selectConfiguration("Control");
  //    scs.selectConfiguration("InitalStudy");



    scs.setDT(DT, 200);

    Thread thread = new Thread(scs);
    thread.setName("SimulationConstructionSet");
    thread.start();
  }

  private void setUpGUI(SimulationConstructionSet scs)
  {
    CameraConfiguration view1 = new CameraConfiguration("view1");
    view1.setCameraDolly(false, true, true, false);
    view1.setCameraTracking(false, true, true, false);
    view1.setCameraPosition(9.2, -37.9, 9.6);
    view1.setCameraFix(0.03, 0.008, 0.18);
    scs.setupCamera(view1);

    scs.selectCamera("view1");

    scs.setupGraphGroup("Control", new String[][][]
                        {
                        {{"t"}, {"auto"}},
                        {{"tauApplied", "q_mass"}, {"auto"}},
                        {{"tauApplied", "qd_mass"}, {""}},
    }, 1);

    scs.setupEntryBoxGroup("Control", new String[]
                           {""});
    //
    scs.setupConfiguration("Control", "all", "Control", "Control");
    //
    //  scs.setupConfiguration("Control", "all", "Control", "Control");
  }


  public static void main(String[] args)
  {
    SimpleSecondOrderSimulation simpleSecondOrderSimulation = new SimpleSecondOrderSimulation();
  }

}
