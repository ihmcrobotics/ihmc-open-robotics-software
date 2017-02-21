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

import java.awt.Dimension;
import java.awt.Graphics;

/**
 * <p>
 * Component for showing spacial information from a sensor.  Typically this information will be 2D in nature.
 * Provides a conversion from world units to pixels for rendering purposes and several optional built in overlays.
 * </p>
 * <p/>
 * <p>
 * Optional Overlays:<br>
 * - scale bar showing the size of some unit<br>
 * - coordinate frame axis.<br>
 * - Everything that {@link InfoDisplay} can overlay
 * </p>
 *
 * @author Peter Abeles
 */
public class SpacialDisplay extends InfoDisplay {

	// conversion between meters and pixels
	protected double metersToPixels = 20.0;

	// draws the scale in the component
	private ScaleComponent scaleComp;

	// draws the coordinate system
	private CoordinateAxisComponent axisComp;

	/**
	 * Adds a basic scale bar showing the length of the provided units.
	 *
	 * @param lengthInPixels If the length is in pixels or distance units.
	 * @param length         How wide the scale bar should be.
	 */
	public void showScale(boolean lengthInPixels, double length) {
		setScaleComponent(new ScaleComponent(lengthInPixels, length));
	}

	/**
	 * Specifies the ScaleComponent that draws the scale bar.  This is used if a fancier or custome scale bar
	 * is desired.
	 *
	 * @param scaleComp
	 */
	public void setScaleComponent(ScaleComponent scaleComp) {
		if (this.scaleComp != null) {
			remove(this.scaleComp);
		}

		this.scaleComp = scaleComp;
		scaleComp.setOwner(this);
	}

	/**
	 * Sets the labels for each axis and the direction of each axis.
	 *
	 * @param labelVert     Label for vertical axis.
	 * @param positiveUp    If the vertical axis is pointing up or down.
	 * @param labelHoriz    Label for horizontal axis.
	 * @param positiveRight If the horizontal axis is pointing left or right.
	 */
	public void showCoordinateAxis(String labelVert, boolean positiveUp,
								   String labelHoriz, boolean positiveRight) {
		axisComp = new CoordinateAxisComponent();
		axisComp.configureAxis(labelVert, positiveUp, labelHoriz, positiveRight);
		axisComp.setSize(new Dimension(60, 60));
		axisComp.setPreferredSize(getSize());
	}

	/**
	 * Specifies a custom coordinate frame axis component.
	 *
	 * @param axisComp The custom component
	 */
	public void setAxisComponent(CoordinateAxisComponent axisComp) {
		this.axisComp = axisComp;
	}

	/**
	 * Adds overlays which show the number of pixels per unit and the 2D coordinate system being used.
	 * The parent overlay function is called.
	 *
	 * @param g      Graphics that the overlays are to be drawn inside of.
	 * @param width  Width of the area that the overlays can be drawn in.
	 * @param height Height of the area that the overlays can be drawn in.
	 */
	@Override
	protected void drawOverlay(Graphics g, int width, int height) {
		if (scaleComp != null) {
			Graphics localG = g.create(5, height - scaleComp.getHeight() - 5, scaleComp.getWidth(), scaleComp.getHeight());
			scaleComp.paintComponent(localG);
		}

		if (axisComp != null) {
			Graphics localG = g.create(0, 0, axisComp.getWidth(), axisComp.getHeight());
			axisComp.paintComponent(localG);
		}

		super.drawOverlay(g, width, height);
	}

	/**
	 * Removes the scale bar from view.
	 */
	public void removeScaleComponent() {
		if (scaleComp != null) {
			super.remove(scaleComp);
			scaleComp = null;
		}
	}

	/**
	 * How many pixels one meter is.
	 *
	 * @return How many pixels one meter is.
	 */
	public double getPixelsPerMeter() {
		return metersToPixels;
	}

	/**
	 * Sets the number of pixels in a meter.
	 *
	 * @param pixelsPerMeter Number of pixels in a meter is the display showing.
	 */
	public void setPixelsPerMeter(double pixelsPerMeter) {
		this.metersToPixels = pixelsPerMeter;
	}
}
