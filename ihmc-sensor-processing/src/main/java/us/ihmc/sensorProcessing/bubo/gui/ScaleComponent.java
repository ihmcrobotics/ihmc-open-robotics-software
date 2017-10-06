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

package us.ihmc.sensorProcessing.bubo.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;

import javax.swing.JComponent;

/**
 * Draws a basic horizontal 'I' bar with text unit indicating the scale of the image in the window.
 *
 * @author Peter Abeles
 */
public class ScaleComponent extends JComponent {

	// the component which this is being drawn in
	private SpacialDisplay owner;

	// How the length is specified
	private boolean lengthByPixels = false;
	// the length of the bar in either distance units or by pixels
	private double length = 1.0;

	// height of the end bars in the 'I'
	private int endHeight = 12;

	// what color will everything be drawn.
	private Color color = Color.BLACK;

	// the units that will be displayed
	private String unitsText = "m";

	// stroke used to draw scale bar
	private Stroke stroke = new BasicStroke(2.0f);

	// spacing used to spread out some components
	private int space = 5;

	/**
	 * @param length Size of the unit bar.
	 */
	public ScaleComponent(boolean lengthByPixels, double length) {
		this();
		this.lengthByPixels = lengthByPixels;
		this.length = length;
	}

	public ScaleComponent() {

	}

	/**
	 * Sets the length of the unit bar.
	 *
	 * @param lengthByPixels If it is specified in pixels or distance units
	 * @param length         The unit bar's new length in pixels or distance units.
	 */
	public void setLength(boolean lengthByPixels, double length) {
		this.length = length;
	}

	public Color getColor() {
		return color;
	}

	public void setColor(Color color) {
		this.color = color;
	}

	/**
	 * Sets the owner of the component.  The metersToPixel conversion is grabbed each time this is shown.
	 *
	 * @param owner
	 */
	public void setOwner(SpacialDisplay owner) {
		this.owner = owner;

		int width = (int) Math.ceil(owner.getPixelsPerMeter() * length) + 5;
		int height = endHeight + 10;

		Dimension d = new Dimension(width, height);
		setSize(d);
		setMinimumSize(d);
		setPreferredSize(d);
	}

	@Override
	public void paintComponent(Graphics g) {

		Graphics2D g2 = (Graphics2D) g;

		g2.setStroke(stroke);

		int height = getHeight();

		double pixelsPerMeter = owner.getPixelsPerMeter();

		int pixelsWide;
		double lengthDistance;

		if (lengthByPixels) {
			pixelsWide = (int) length;
			lengthDistance = length / pixelsPerMeter;
		} else {
			pixelsWide = (int) Math.round(pixelsPerMeter * length);
			lengthDistance = length;
		}

		g2.setColor(color);
//
		// draw the end lines
		g2.drawLine(space, height - endHeight, space, height);
		g2.drawLine(space+pixelsWide, height - endHeight, space+pixelsWide, height);

		// draw the center line
		g2.drawLine(space, height - endHeight / 2, space+pixelsWide, height - endHeight / 2);

		// todo compute text size with TextLayout and put in center of the bar
		String text = String.format("%3.1f " + unitsText, lengthDistance);
		g2.drawString(text, 5 + space, height - endHeight / 2 - 5);
	}

	public Stroke getStroke() {
		return stroke;
	}

	public void setStroke(Stroke stroke) {
		this.stroke = stroke;
	}
}
