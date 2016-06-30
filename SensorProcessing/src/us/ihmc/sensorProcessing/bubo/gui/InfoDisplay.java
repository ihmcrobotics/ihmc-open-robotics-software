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

import java.awt.Graphics;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JComponent;

/**
 * A component that is designed to add some common functionality that is needed when displaying information to users.
 * For example, it makes it very easy to add a logo to a component. Several classes extend this one to provide
 * functionality for specific types of information.
 *
 * @author Peter Abeles
 */
// TODO add source text
// TODO add time stamp
public class InfoDisplay extends JComponent {

	// logo configuration
	private List<LogoComponent> logos = new ArrayList<LogoComponent>();

	public InfoDisplay() {

	}

	@Override
	public void paint(Graphics g) {
		super.paint(g);

		Rectangle view = getVisibleRect();
		Graphics overlayG = g.create(view.x, view.y, view.width, view.height);
		drawOverlay(overlayG, view.width, view.height);
	}

	/**
	 * Draws overlay objects after all the other objects have been drawn so that they are always
	 * visible. Unless overloaded this function will draw logo's starting in the lower right hand corner.
	 *
	 * @param g      Graphics that the overlays are to be drawn inside of.
	 * @param width  Width of the area that the overlays can be drawn in.
	 * @param height Height of the area that the overlays can be drawn in.
	 */
	protected void drawOverlay(Graphics g, int width, int height) {
		int x = width;

		for (JComponent c : logos) {
			int y = height - c.getHeight() - 5;
			x -= c.getWidth() + 5;

			Graphics componentG = g.create(x, y, c.getWidth(), c.getHeight());
			c.paint(componentG);
		}
	}

	public void addLogo(LogoComponent c) {
		logos.add(c);
	}


	public void clearLogos() {
		logos.clear();
	}
}
