package us.ihmc.valkyrie.testsupport;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.simulationConstructionSetTools.dataExporter.DataExporterExcelWorkbookCreator;
import us.ihmc.simulationConstructionSetTools.dataExporter.TorqueSpeedDataExporterGraphCreator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ValkyrieTestExporter {
	

	private static void writeReadme(File readmeFile, String info)
	{
		try
		{
			FileWriter out = new FileWriter(readmeFile);
			out.write(info);
			out.close();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}	
	
	// Export sim data, graphs and video, if available
	public static File exportSimData(SimulationConstructionSet scs, File dataParentFolder, String dataNameSuffix, String info, boolean simSucceeded)
	{
		Robot robot = scs.getRobots()[0];
		TorqueSpeedDataExporterGraphCreator graphCreator = new TorqueSpeedDataExporterGraphCreator(robot, scs.getDataBuffer());
		DataExporterExcelWorkbookCreator excelWorkbookCreator = new DataExporterExcelWorkbookCreator(robot, scs.getDataBuffer());
		
		if (!simSucceeded) {
			info += "_FAILED";
			dataNameSuffix += "_FAILED";
		}

		// Stop the sim and disable the GUI:
		scs.stop();
		scs.disableGUIComponents();

		// Wait till done running:
		while (scs.isSimulating())
		{
			ThreadTools.sleep(1000);
		}

		// Crop the Buffer to In/Out. This is important because of how we use the DataBuffer later and we assume that in point is at index=0:
		scs.cropBuffer();
		scs.gotoInPointNow();

		String timeStamp = FormattingTools.getDateString() + "_" + FormattingTools.getTimeStringWithSeconds();
		String tagName = timeStamp + "_" + robot.getName() + "_" + dataNameSuffix;

		File dataFolder = new File(dataParentFolder, tagName);
		dataFolder.mkdir();

		System.out.println("Saving ReadMe");
		writeReadme(new File(dataFolder, tagName + ".txt"), info);
		System.out.println("Done Saving ReadMe");

		System.out.println("Saving data");
		scs.writeMatlabData("all", new File(dataFolder, tagName + ".mat"));
		System.out.println("Done Saving Data");

		System.out.println("Saving data in Matlab format");
		try
		{
			scs.writeMatlabData("all", new File(dataFolder, tagName + ".m"));
			System.out.println("Done Saving Data in Matlab format");
		}
		catch (OutOfMemoryError exception)
		{
			System.err.println("Ran out of memory while saving to Matlab format. Try again with fewer points.");
			exception.printStackTrace();
		}

		System.out.println("creating torque and speed spreadsheet");
		excelWorkbookCreator.createAndSaveTorqueAndSpeedSpreadSheet(dataFolder, tagName);
		System.out.println("done creating torque and speed spreadsheet");

		System.out.println("creating torque and speed graphs");
		// make graph directory inside destination directory
		File graphDirectory = new File(dataFolder, "graphs");
		graphDirectory.mkdir();
		graphCreator.createJointTorqueSpeedGraphs(graphDirectory, tagName, true, false);
		System.out.println("done creating torque and speed graphs");

		if (scs.getSimulationConstructionSetParameters().getCreateGUI()) {
			System.out.println("creating video");
			scs.getStandardSimulationGUI().getViewportPanel().getStandardGUIActions().createVideo(new File(dataFolder, tagName + "_Video.mov"));
			System.out.println("done creating video");
		}

		scs.enableGUIComponents();

		return dataFolder;
	}

}
