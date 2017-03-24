package us.ihmc.simulationconstructionsettools.dataExporter;

import static org.junit.Assert.*;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class DataExporterGraphCreatorTest
{

    @ContinuousIntegrationTest(estimatedDuration = 1.0)
    @Test(timeout = 30000)
    public void testDataExporterGraphCreator() throws IOException
    {
        SimulationConstructionSet sim = createSimulation();
        TorqueSpeedDataExporterGraphCreator graphCreator = new TorqueSpeedDataExporterGraphCreator(sim.getRobots()[0], sim.getDataBuffer());

        File path = new File(System.getProperty("java.io.tmpdir"));
        Path tmpPath = Files.createTempDirectory(Paths.get(path.getAbsolutePath()), "test");
        graphCreator.createJointTorqueSpeedGraphs(tmpPath.toFile(), "", true, true);

        int fileCount = tmpPath.toFile().listFiles(new FilenameFilter()
            {
                @Override
                public boolean accept(File dir, String name)
                {
                    return name.endsWith("jpg") || name.endsWith("pdf");
                }
            }).length;
        assertTrue(fileCount > 1);

        tmpPath.toFile().deleteOnExit();
    }


    private SimulationConstructionSet createSimulation()
    {
        SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
        parameters.setCreateGUI(false);
        return new SimulationConstructionSet(createRobot(), parameters);
    }

    private Robot createRobot()
    {
        Robot robot = new Robot("RandomRobot");

        PinJoint pinJoint = new PinJoint("TestPinJoint", new Vector3D(), robot, new Vector3D(0, 0, 1));
        robot.addRootJoint(pinJoint);

        return robot;
    }
}