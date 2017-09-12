package us.ihmc.manipulation.planning.rrt.constrainedplanning.tools;

import java.awt.Color;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CTTaskNodeTreeVisualizer {
	private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

	private final SimulationConstructionSet scs;
	private CTTaskNodeTree taskNodeTree;

	private static boolean showNormalized = true;

	public CTTaskNodeTreeVisualizer(SimulationConstructionSet scs, CTTaskNodeTree tree) {
		this.scs = scs;
		this.taskNodeTree = tree;
	}

	public void visualize() {
		int dimensionOfTask = taskNodeTree.getDimensionOfTask();

		for (int i = 1; i < dimensionOfTask + 1; i++) {
			if (taskNodeTree.getTaskNodeRegion().isEnable(i)) {
				SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs
						.createSimulationOverheadPlotterFactory();

				plottingTaskTree(simulationOverheadPlotterFactory, i);
			}
		}
	}

	private void plottingTaskTree(SimulationOverheadPlotterFactory simulationOverheadPlotterFactory,
			int indexOfDimension) {
		YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

		/*
		 * valid nodes
		 */
		for (int i = 0; i < taskNodeTree.getWholeNodes().size(); i++) {
			CTTaskNode node = taskNodeTree.getWholeNodes().get(i);
			String prefix = taskNodeTree.getTaskName(indexOfDimension) + "_node_" + i;
			if (node.getParentNode() == null) {
				PrintTools.info("whole node : this is root node");
				yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_node",
						createNode(node, indexOfDimension, prefix, 0));
			} else {
				yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_node",
						createNode(node, indexOfDimension, prefix, 0));
				yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_branch",
						createBranch(node, indexOfDimension, prefix));
			}
		}
		/*
		 * fail nodes
		 */

		for (int i = 0; i < taskNodeTree.getFailNodes().size(); i++) {
			CTTaskNode node = taskNodeTree.getFailNodes().get(i);
			String prefix = taskNodeTree.getTaskName(indexOfDimension) + "_fail_node_" + i;

			yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_node",
					createNode(node, indexOfDimension, prefix, 1));
		}

		/*
		 * path nodes
		 */
		if (taskNodeTree.getPath() != null) {
			for (int i = 0; i < taskNodeTree.getPath().size(); i++) {
				CTTaskNode node = taskNodeTree.getPath().get(i);
				String prefix = taskNodeTree.getTaskName(indexOfDimension) + "_path_" + i;
				if (node.getParentNode() == null) {
					PrintTools.info("path : this is root node");
					yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_path",
							createNode(node, indexOfDimension, prefix, 2));
				} else {
					yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_path",
							createNode(node, indexOfDimension, prefix, 2));
					yoGraphicsListRegistry.registerArtifact("" + prefix + "_artifact_pathbranch",
							createBranch(node, indexOfDimension, prefix));
				}
			}
		}
		simulationOverheadPlotterFactory.setPlotterName("Task Tree Plotter : " + taskNodeTree.getTaskName(indexOfDimension));
		simulationOverheadPlotterFactory.setVariableNameToTrack("");
		simulationOverheadPlotterFactory.setShowOnStart(true);
		simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
		simulationOverheadPlotterFactory.setCreateInSeperateWindow(true);
		simulationOverheadPlotterFactory.createOverheadPlotter();
	}

	private YoArtifactLineSegment2d createBranch(CTTaskNode taskNode, int indexOfDimension, String prefix) {
		YoFrameLineSegment2d yoLine = new YoFrameLineSegment2d("" + prefix + "_line", "",
				ReferenceFrame.getWorldFrame(), registry);

		FramePoint2D nodePoint;
		FramePoint2D parentNodePoint;

		if (showNormalized) {
			nodePoint = new FramePoint2D(ReferenceFrame.getWorldFrame(), taskNode.getNormalizedNodeData(0),
					taskNode.getNormalizedNodeData(indexOfDimension));
			parentNodePoint = new FramePoint2D(ReferenceFrame.getWorldFrame(),
					taskNode.getParentNode().getNormalizedNodeData(0),
					taskNode.getParentNode().getNormalizedNodeData(indexOfDimension));
		} else {
			nodePoint = new FramePoint2D(ReferenceFrame.getWorldFrame(), taskNode.getNodeData(0),
					taskNode.getNodeData(indexOfDimension));
			parentNodePoint = new FramePoint2D(ReferenceFrame.getWorldFrame(), taskNode.getParentNode().getNodeData(0),
					taskNode.getParentNode().getNodeData(indexOfDimension));
		}

		yoLine.set(nodePoint, parentNodePoint);

		YoArtifactLineSegment2d artifactLine = new YoArtifactLineSegment2d("" + prefix + "_branch", yoLine,
				Color.BLACK);

		return artifactLine;
	}

	/*
	 * 0 = valid 1 = fail 2 = path
	 */
	private YoArtifactOval createNode(CTTaskNode taskNode, int indexOfDimension, String prefix, int type) {
		YoFramePoint yoPoint = new YoFramePoint("" + prefix, ReferenceFrame.getWorldFrame(), registry);
		if (showNormalized) {
			yoPoint.setX(taskNode.getNormalizedNodeData(0));
			yoPoint.setY(taskNode.getNormalizedNodeData(indexOfDimension));
		} else {
			yoPoint.setX(taskNode.getNodeData(0));
			yoPoint.setY(taskNode.getNodeData(indexOfDimension));
		}

		YoDouble radius = new YoDouble("" + prefix, registry);

		YoArtifactOval artifactOval = new YoArtifactOval("" + prefix, yoPoint, radius, Color.BLUE);
		switch (type) {
		case 0:
			artifactOval = new YoArtifactOval("" + prefix, yoPoint, radius, Color.BLUE);
			radius.set(0.02);
			break;
		case 1:
			artifactOval = new YoArtifactOval("" + prefix, yoPoint, radius, Color.RED);
			radius.set(0.02);
			break;
		case 2:
			artifactOval = new YoArtifactOval("" + prefix, yoPoint, radius, Color.GREEN);
			radius.set(0.03);
			break;
		}

		return artifactOval;
	}

}
