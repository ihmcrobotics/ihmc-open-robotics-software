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
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;

import us.ihmc.sensorProcessing.bubo.gui.SpacialDisplay;

/**
 * @author Peter Abeles
 */
public class LaserHistogram extends SpacialDisplay implements ComponentListener {

	// number of range measurements
	private int numMeas;

	// measured range
	private double ranges[] = new double[1];

	// the maximum allowed range
	private double maxRange;

	private Color colorBack = Color.WHITE;
	private Color colorLine = Color.BLACK;
	private Color colorInvalid = Color.CYAN;
	private Color colorMaxRange = Color.BLUE;

	public LaserHistogram() {
		addComponentListener(this);
		showScale(true, 60);
	}

	public void setData(int numMeas, float[] data, float maxRange) {
		// adjust the scale for display
		setPixelsPerMeter(getHeight() / maxRange);

		if (ranges.length < numMeas) {
			ranges = new double[numMeas];
		}

		for (int i = 0; i < numMeas; i++) {
			ranges[i] = data[i];
		}
		this.numMeas = numMeas;
		this.maxRange = maxRange;

		setBackground(colorBack);

		repaint();
	}

	public void setData(int numMeas, double[] data, double maxRange) {
		// adjust the scale for display
		setPixelsPerMeter(getHeight() / maxRange);

		if (ranges.length < numMeas) {
			ranges = new double[numMeas];
		}

		System.arraycopy(data, 0, ranges, 0, numMeas);

		this.numMeas = numMeas;
		this.maxRange = maxRange;

		setBackground(colorBack);

		repaint();
	}

	@Override
	public void paintComponent(Graphics g) {
		Graphics2D g2 = (Graphics2D) g;

		int width = getWidth();
		int height = getHeight();

		for (int i = 0; i < numMeas; i++) {
			int x = i * width / numMeas;
			int x_next = (i + 1) * width / numMeas;

			double r = ranges[i];

			if (Double.isNaN(r)) {
				g2.setColor(colorInvalid);
				g2.fillOval(x, height - 4, 4, 4);
			} else if (r >= maxRange) {
				g2.setColor(colorMaxRange);
				g2.fillOval(x, height - 4, 4, 4);
			} else {
				int y = (int) (height * r / maxRange);
				g2.setColor(colorLine);
				g2.fillRect(x, height - y, (x_next - x), y);
			}
		}
	}

	@Override
	public void componentResized(ComponentEvent e) {
		if (maxRange > 0) {
			setPixelsPerMeter(getHeight() / maxRange);
		}
	}

	@Override
	public void componentMoved(ComponentEvent e) {
	}

	@Override
	public void componentShown(ComponentEvent e) {
	}

	@Override
	public void componentHidden(ComponentEvent e) {
	}
}
