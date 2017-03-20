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
 * Component for visualizing data from a 2D laser ranging sensor.  Lines that represent the range of each measurement
 * are drawn radially outward from the center of the image.  Angle zero is along the x-axis with the y-axis being 90 degrees.
 * <p/>
 * If a max range is set then measurements that are beyond that are ignored.
 * <p/>
 * Can be configured to automatically change the pixels per meter of the component so that it always
 * displays the max range.
 *
 * @author Peter Abeles
 */
public class LadarComponent extends SpacialDisplay implements ComponentListener {

	// number of range measurements
	private int numMeas;

	// measured range
	private double ranges[];
	// sine and cosine of each angle
	private double cos[];
	private double sin[];

	// if not zero this is the maximum range of the sensor.  Any value beyond
	// is considered invalid.
	private double maxRange;

	// if set to true it will rescale so that the display is always visible
	private boolean autoRescale = false;

	/**
	 * Each laser scan is a constant number of radians away from the previous scan in consecutive order.
	 *
	 * @param startAngle The angle the first laser is at.
	 * @param deltaAngle Number of radians away each ladar scan is away from the previous.
	 * @param maxRange   If not zero it specifies the maximum valid range of the sensor.
	 * @param numMeas    Number of ladar scans.
	 */
	public LadarComponent(double startAngle, double deltaAngle, double maxRange, int numMeas) {
		super();
		configure(startAngle, deltaAngle, maxRange, numMeas);
	}

	public LadarComponent() {
		showScale(true, 60);
		setPixelsPerMeter(10.0);
		showCoordinateAxis("Y", true, "X", true);
	}

	public void configure(double startAngle, double deltaAngle,
						  double maxRange, int numMeas) {
		boolean maxRangeChanged = this.maxRange != maxRange;

		this.numMeas = numMeas;
		this.maxRange = maxRange;

		if (ranges == null || ranges.length < numMeas) {
			ranges = new double[numMeas];
			cos = new double[numMeas];
			sin = new double[numMeas];
		}

		double a = startAngle;
		for (int i = 0; i < numMeas; i++, a += deltaAngle) {
			cos[i] = Math.cos(a);
			sin[i] = Math.sin(a);
		}

		// update the scaling for the new max range
		if (maxRangeChanged) {
			componentResized(null);
		}
	}

	/**
	 * Automatically rescales the component such that the measurements of the max range are always
	 * visible.
	 *
	 * @param autoRescale If rescaling is turned on or off.
	 */
	public void setAutoRescale(boolean autoRescale) {
		if (!this.autoRescale && autoRescale) {
			addComponentListener(this);
		} else if (!autoRescale) {
			removeComponentListener(this);
		}

		this.autoRescale = autoRescale;

		// change the scale if needed
		componentResized(null);
	}

	public void setData(float[] data) {
		for (int i = 0; i < numMeas; i++) {
			double d = data[i];
			if (Double.isNaN(d) || Double.isInfinite(d)) {
				ranges[i] = 0;
			} else if (d > maxRange)
				ranges[i] = 0;
			else
				ranges[i] = data[i];
		}

		repaint();
	}


	/**
	 * Sets the laser data.
	 *
	 * @param data data that is to be displayed.
	 */
	public void setData(double[] data) {
		for (int i = 0; i < numMeas; i++) {
			double d = data[i];
			if (Double.isNaN(d) || Double.isInfinite(d)) {
				ranges[i] = 0;
			} else if (d > maxRange)
				ranges[i] = 0;
			else
				ranges[i] = data[i];
		}
	}

	public double[] getRanges() {
		return ranges;
	}

	@Override
	public void paintComponent(Graphics g) {
		Graphics2D g2 = (Graphics2D) g;

		// clean up the background
		g2.setColor(Color.white);
		g2.fillRect(0, 0, getWidth(), getHeight());

		int centerX = getWidth() / 2;
		int centerY = getHeight() / 2;

		g2.setColor(Color.black);
		for (int i = 0; i < numMeas; i++) {

			double r = ranges[i];
			if (r <= 0)
				continue;

			if (r < maxRange) {

				int destX = centerX + (int) (cos[i] * ranges[i] * metersToPixels);
				int destY = centerY - (int) (sin[i] * ranges[i] * metersToPixels);

				g2.drawLine(centerX, centerY, destX, destY);
			}
		}
	}

	/**
	 * Changes the components scale so that the same area is being displayed
	 */
	@Override
	public void componentResized(ComponentEvent e) {
		double r = maxRange;

		if (r > 0) {
			int length = Math.min(getWidth(), getHeight());
			double pixelsPerMeter = length / (3.0 * r);
			// only change it if needed
			if (pixelsPerMeter != getPixelsPerMeter())
				setPixelsPerMeter(pixelsPerMeter);
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
