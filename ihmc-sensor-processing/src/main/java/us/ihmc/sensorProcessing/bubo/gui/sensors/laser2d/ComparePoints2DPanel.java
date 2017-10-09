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

package us.ihmc.sensorProcessing.bubo.gui.sensors.laser2d;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

import georegression.struct.point.Point2D_F64;
import us.ihmc.sensorProcessing.bubo.gui.SpacialDisplay;

/**
 * Displays multiple sets of 2D points in color
 *
 * @author Peter Abeles
 */
public class ComparePoints2DPanel extends SpacialDisplay {

	List<List<Point2D_F64>> sets = new ArrayList<List<Point2D_F64>>();
	List<Color> colors = new ArrayList<Color>();

	double gridWidth = 2.0;
	Color gridColor =  new Color(220, 220, 220);

	public ComparePoints2DPanel() {
		showScale(true, 60);
	}

	public void addPoints( List<Point2D_F64> points , Color color ) {
		sets.add(points);
		colors.add(color);
	}

	public void clear() {
		sets.clear();
		colors.clear();
	}

	@Override
	public void paintComponent(Graphics g) {

		Graphics2D g2 = (Graphics2D)g;

		drawGrid(g2);
		drawPoints(g2);
	}

	private void drawPoints(Graphics2D g2) {
		int centerX = getWidth()/2;
		int centerY = getHeight()/2;
		int r = 1;
		int w = r*2+1;

		for (int i = 0; i < sets.size(); i++) {
			g2.setColor(colors.get(i));
			List<Point2D_F64> points = sets.get(i);

			for (int k = 0; k < points.size(); k++) {
				Point2D_F64 p = points.get(k);
				int x = (int)Math.round(p.getX()*metersToPixels) + centerX;
				int y = (int)Math.round(p.getY()*metersToPixels) + centerY;

				g2.fillOval(x-r,y-r,w,w);
			}
		}
	}

	private void drawGrid( Graphics2D g2 ) {

		int width = getWidth();
		int height = getHeight();

		g2.setColor(gridColor);

		double viewWidth = width/metersToPixels;
		double viewHeight = height/metersToPixels;

		double startX = Math.floor((-viewWidth/2)/gridWidth)*gridWidth;
		double startY = Math.floor((-viewHeight/2)/gridWidth)*gridWidth;

		int numCols = (int)(viewWidth/gridWidth)+2;
		int numRows = (int)(viewHeight/gridWidth)+2;

		for (int i = 0; i < numRows; i++) {
			int y = (int)Math.round((startY+i*gridWidth)*metersToPixels)+height/2;

			g2.drawLine(0, height - 1 - y, width, height - 1 - y);
		}

		for (int i = 0; i < numCols; i++) {
			int x = (int)Math.round((startX+i*gridWidth)*metersToPixels)+width/2;
			g2.drawLine(x,0,x,height);
		}
	}

	public void setGridWidth(double gridWidth) {
		this.gridWidth = gridWidth;
	}
}
