package us.ihmc.simulationconstructionset.dataExporter;

import org.junit.Test;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.testing.TestPlanAnnotations;

import javax.vecmath.Vector3d;
import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.junit.Assert.assertTrue;


public class DataExporterGraphCreatorTest
{

    @Test
    @TestPlanAnnotations.DeployableTestMethod
    public void testDataExporterGraphCreator() throws IOException
    {
        SimulationConstructionSet sim = createSimulation();
        DataExporterGraphCreator graphCreator = new DataExporterGraphCreator(sim.getRobots()[0], sim.getDataBuffer());

        File path = new File(System.getProperty("java.io.tmpdir"));
        Path tmpPath = Files.createTempDirectory(Paths.get(path.getAbsolutePath()), "test");
        graphCreator.createGraphs(tmpPath.toFile(), "", true, true);

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

        PinJoint pinJoint = new PinJoint("TestPinJoint", new Vector3d(), robot, new Vector3d(0, 0, 1));
        robot.addRootJoint(pinJoint);

        return robot;
    }
}