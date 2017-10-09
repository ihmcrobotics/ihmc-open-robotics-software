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
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;
import java.awt.font.FontRenderContext;
import java.awt.font.TextLayout;
import java.awt.geom.Rectangle2D;

import javax.swing.JComponent;

/**
 * Draws a 2D coordinate frame axis.  Arrows point in the direction of each axis and each axis can be labeled.
 *
 * @author Peter Abeles
 */
public class CoordinateAxisComponent extends JComponent {

	// label of each axis
	private String labelY = "Y"; // vertical axis
	private String labelX = "X"; // horizontal axis

	// is the vertical axis positive in the upwards direction
	private boolean positiveUp = true;
	// is the horizontal axis positive to the right
	private boolean positiveRight = true;

	// default value for a buffer between stuff being drawn
	private int space = 5;

	// stuff that can change the appearance of the coordinate frame axis.
	private Font font = new Font("Helvetica", Font.BOLD, 14);
	private Color c = Color.BLACK;
	private Stroke stroke = new BasicStroke(2.0f);

	public CoordinateAxisComponent() {
		setSize(50, 50);
		setPreferredSize(getSize());
		setMinimumSize(getSize());
	}

	/**
	 * Sets the labels for each axis and the direction of each axis.
	 *
	 * @param labelVert     Label for vertical axis.
	 * @param positiveUp    If the vertical axis is pointing up or down.
	 * @param labelHoriz    Label for horizontal axis.
	 * @param positiveRight If the horizontal axis is pointing left or right.
	 */
	public void configureAxis(String labelVert, boolean positiveUp,
							  String labelHoriz, boolean positiveRight) {
		labelY = labelVert;
		labelX = labelHoriz;
		this.positiveUp = positiveUp;
		this.positiveRight = positiveRight;
	}

	@Override
	public void paintComponent(Graphics g) {

		Graphics2D g2 = (Graphics2D) g;

		g2.setColor(c);
		g2.setStroke(stroke);

		FontRenderContext frc = g2.getFontRenderContext();
		TextLayout tlX = new TextLayout(labelX, font, frc);
		TextLayout tlY = new TextLayout(labelY, font, frc);

		Rectangle2D boundsX = tlX.getBounds();
		Rectangle2D boundsY = tlY.getBounds();

		int width = getWidth();
		int height = getHeight();

		float originX = positiveRight ? (float) boundsY.getWidth() + space : width - (float) boundsY.getWidth() - space;
		float originY = positiveUp ? height - (float) boundsX.getHeight() - space : space + (float) boundsX.getHeight();

		float endY = positiveUp ? space : height - space;
		float endX = positiveRight ? width - space : space;

		// draw tha coordinate frame axis
		DrawShapes.arrow(g2, originX, originY, originX, endY, 10f, 0.4f);
		DrawShapes.arrow(g2, originX, originY, endX, originY, 10f, 0.4f);

		// draw each label in the middle of their respective axis
		if (positiveRight)
			tlY.draw(g2, 0f,
					(endY + originY) / 2.0f + (float) (boundsY.getHeight() / 2.0));
		else
			tlY.draw(g2, originX + space,
					(endY + originY) / 2.0f + (float) (boundsY.getHeight() / 2.0));

		if (positiveUp)
			tlX.draw(g2, (endX + originX) / 2.0f - (float) (boundsX.getWidth() / 2.0),
					originY + (float) boundsX.getHeight() + space);
		else
			tlX.draw(g2, (endX + originX) / 2.0f - (float) (boundsX.getWidth() / 2.0),
					(float) boundsX.getHeight());
	}

	public Stroke getStroke() {
		return stroke;
	}

	/**
	 * Sets the stroke that the lines in the arrows are drawn with
	 */
	public void setStroke(Stroke stroke) {
		this.stroke = stroke;
	}

	public Color getC() {
		return c;
	}

	/**
	 * Sets the color of everything being drawn.
	 *
	 * @param c
	 */
	public void setC(Color c) {
		this.c = c;
	}

	public Font getFont() {
		return font;
	}

	/**
	 * Sets the font the axis labels are drawn with.
	 */
	public void setFont(Font font) {
		this.font = font;
	}
}
