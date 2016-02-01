package us.ihmc.moonwalking.models.PassiveDynamicWalker;

import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
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
public class PassiveDynamicWalkerSimulation
{
    private SimulationConstructionSet sim;
    private PassiveDynamicWalkerRobot passiveDynamicWalker;
    public static final  double GROUND_ANGLE_OF_INCLINATION = 0.036;

    private final YoVariableRegistry registry = new YoVariableRegistry("Simulation");
    private final YoVariable K_XY = new YoVariable("K_XY", registry);
    private final YoVariable B_XY = new YoVariable("B_XY", registry);
    private final YoVariable K_Z = new YoVariable("K_Z", registry);
    private final YoVariable B_Z = new YoVariable("B_Z", registry);

    public PassiveDynamicWalkerSimulation()
    {
        passiveDynamicWalker = new PassiveDynamicWalkerRobot();
        PassiveDynamicWalkerController passiveDynamicWalkerController = new PassiveDynamicWalkerController(passiveDynamicWalker);
        passiveDynamicWalker.setController(passiveDynamicWalkerController);
        GroundProfile inclinedGroundProfile = new InclinedGroundProfile(GROUND_ANGLE_OF_INCLINATION);

        K_XY.val = 15000.0;
        B_XY.val = 2000.0;
        K_Z.val = 500.0;
        B_Z.val = 1000.0;

        GroundContactModel linearGroundContactModel = new LinearGroundContactModel(passiveDynamicWalker);
//        GroundContactModel linearGroundContactModel = new LinearGroundContactModel(passiveDynamicWalker, K_XY, B_XY, K_Z, B_Z);


        linearGroundContactModel.setGroundProfile(inclinedGroundProfile);
        passiveDynamicWalker.setGroundContactModel(linearGroundContactModel);

        sim = new SimulationConstructionSet(passiveDynamicWalker);
        sim.setDT(0.00001, 100);

        sim.addVarList(registry.createVarList());

        Thread myThread = new Thread(sim);
        myThread.start();
    }

    public static void main(String[] args)
    {
        PassiveDynamicWalkerSimulation sim = new PassiveDynamicWalkerSimulation();
    }
}
