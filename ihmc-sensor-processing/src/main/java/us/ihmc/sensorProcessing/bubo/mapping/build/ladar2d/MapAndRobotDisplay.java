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

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import georegression.struct.se.Se2_F64;
import us.ihmc.sensorProcessing.bubo.gui.maps.GridMapBasicDisplay;

/**
 * Draw a robot on top of the occupancy grid.
 *
 * @author Peter Abeles
 */
public class MapAndRobotDisplay extends GridMapBasicDisplay {

	Se2_F64 robotLoc;
	int radius = 10;
	BasicStroke directionStroke = new BasicStroke(3);

	public void updateRobot(Se2_F64 robotLoc) {
		this.robotLoc = robotLoc.copy();
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);

		if (robotLoc == null)
			return;

		Graphics2D g2 = (Graphics2D) g;

		// global to map coordinates
		double x = robotLoc.getX() - spacial.getBl().x;
		double y = robotLoc.getY() - spacial.getBl().y;

		// map coordinates to image pixel coordinates
		int pixelX = (int) mapToGuiX(x) - radius;
		int pixelY = (int) mapToGuiY(y) - radius;

//        System.out.println("vis rect "+v.x+" "+v.y+" "+v.width+" "+v.height);

		g2.setColor(Color.WHITE);
		g2.drawOval(pixelX, pixelY, radius * 2 + 1, radius * 2 + 1);
		g2.setColor(Color.RED);
		g2.fillOval(pixelX + 1, pixelY + 1, radius * 2 - 1, radius * 2 - 1);

		// draw the robot's orientation
		g2.setColor(Color.BLACK);
		g2.setStroke(directionStroke);
		pixelX += radius;
		pixelY += radius;
		int dx = (int) (robotLoc.getCosineYaw() * radius);
		int dy = (int) (robotLoc.getSineYaw() * radius);
		g2.drawLine(pixelX, pixelY, pixelX + dx, pixelY - dy);
	}
}
