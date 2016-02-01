package us.ihmc.moonwalking.models.RaibertHopper;

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
public class RaibertHopperSimulation
{
    private SimulationConstructionSet sim;
    private RaibertHopperRobot raibertHopper;

    public RaibertHopperSimulation()
    {
        raibertHopper = new RaibertHopperRobot();
        RaibertHopperController raibertHopperController = new RaibertHopperController(raibertHopper);
        raibertHopper.setController(raibertHopperController);
//        raibertHopper.setGroundContactModel(new LinearStickSlipGroundContactModel(raibertHopper, 1422, 15.6, 725, 700, 15, 15));
        raibertHopper.setGroundContactModel(new LinearGroundContactModel(raibertHopper, 15000, 2000, 500, 1000));

        sim = new SimulationConstructionSet(raibertHopper);
        sim.setDT(0.00001, 100);
        Thread myThread = new Thread(sim);
        myThread.start();
    }

    public static void main(String[] args)
    {
        RaibertHopperSimulation sim = new RaibertHopperSimulation();
    }
}
