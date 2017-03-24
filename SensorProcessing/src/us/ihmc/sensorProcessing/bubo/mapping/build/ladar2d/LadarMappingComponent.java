/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.ihmc.sensorProcessing.bubo.mapping.build.ladar2d;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JToolBar;

import georegression.struct.se.Se2_F64;
import us.ihmc.sensorProcessing.bubo.desc.sensors.lrf2d.Lrf2dParam;
import us.ihmc.sensorProcessing.bubo.gui.maps.GridMapInteractionDisplay;
import us.ihmc.sensorProcessing.bubo.gui.sensors.laser2d.LadarComponent;
import us.ihmc.sensorProcessing.bubo.maps.d2.grid.GridMapSpacialInfo;
import us.ihmc.sensorProcessing.bubo.maps.d2.grid.OccupancyGrid2D_F32;

/**
 * @author Peter Abeles
 */
// TODO show data frame index
public class LadarMappingComponent extends JComponent implements ActionListener {

	Lrf2dParam param;
	LadarComponent ladarRaw;

	MapAndRobotDisplay mapDisplay;
	GridMapInteractionDisplay interactionDisplay;

	JButton bPlay;
	JButton bStep;
	JButton bFocusRobot;
	JButton bSaveMapImage;

	public LadarMappingComponent() {
		setLayout(new BorderLayout());
		ladarRaw = new LadarComponent();
		ladarRaw.setAutoRescale(true);
		ladarRaw.setPreferredSize(new Dimension(400, 400));
		ladarRaw.setMinimumSize(ladarRaw.getPreferredSize());

		mapDisplay = new MapAndRobotDisplay();
		mapDisplay.setPixelsPerMeter(20);

		interactionDisplay = new GridMapInteractionDisplay(mapDisplay);

		addToolbar();
		add(interactionDisplay, BorderLayout.CENTER);
		add(ladarRaw, BorderLayout.EAST);
	}

	public JButton getPlayButton() {
		return bPlay;
	}

	public JButton getStepButton() {
		return bStep;
	}

	public JButton getFocusRobotButton() {
		return bFocusRobot;
	}

	public JButton getSaveMapImageButton() {
		return bSaveMapImage;
	}

	/**
	 * Returns the component that is used to render the map and robot.
	 */
	public MapAndRobotDisplay getMapDisplay() {
		return mapDisplay;
	}

	private void addToolbar() {
		JToolBar toolbar = new JToolBar("Tools");

		bPlay = new JButton("Play");
		bStep = new JButton("Step");
		bFocusRobot = new JButton("Focus Robot");
		bSaveMapImage = new JButton("Save Map Image");

		toolbar.add(bPlay);
		toolbar.add(bStep);
		toolbar.add(bFocusRobot);
		toolbar.add(bSaveMapImage);

		add(toolbar, BorderLayout.NORTH);
	}

	public void configureLadar(Lrf2dParam param) {
		this.param = param;
		ladarRaw.configure(param.getStartAngle(), param.getAngleIncrement(),
				param.getMaxRange(), param.getNumberOfScans());
	}

	public void setMap(GridMapSpacialInfo mapSpacial,
					   OccupancyGrid2D_F32 map) {
		mapDisplay.setMap(mapSpacial, map);
	}

	public void updateRobot(Se2_F64 robotLoc) {
		mapDisplay.updateRobot(robotLoc);
	}

	public void updateMap() {
		mapDisplay.mapChanged();
	}

	public void updateLadar(double[] rangeData) {
		ladarRaw.setData(rangeData);
	}

	@Override
	public void actionPerformed(ActionEvent e) {

	}

	/**
	 * Specifies where the center of the view should be focused in map coordinates.
	 *
	 * @param x x-axis in map units.
	 * @param y y-axis in map units.
	 */
	public void setViewCenter(double x, double y) {
		interactionDisplay.setViewCenter((int) mapDisplay.mapToGuiX(x), (int) mapDisplay.mapToGuiY(y));
	}
}
