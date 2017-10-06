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

import java.awt.Graphics2D;
import java.awt.geom.GeneralPath;

/**
 * @author Peter Abeles
 */
public class DrawShapes {

	/**
	 * Draws a basic arrow into the Graphics2D object.
	 *
	 * @param g2         Where the arrow is drawn.
	 * @param startX     start of the arrow.  x-coordinate
	 * @param startY     start of the arrow.  y-coordinate
	 * @param endX       end of the arrow.  x-coordinate
	 * @param endY       end of the arrow.  y-coordinate
	 * @param headLength How long the head is in front of the arrow: 10
	 * @param headAngle  The angle the head lines are relative to the body lines: 0.4
	 */
	public static void arrow(Graphics2D g2, float startX, float startY, float endX, float endY,
							 float headLength, float headAngle) {
		GeneralPath shape = new GeneralPath();

		shape.moveTo(startX, startY);
		shape.lineTo(endX, endY);

		double lineAngle = Math.atan2(endY - startY, endX - startX) + Math.PI;

		double targetX = endX + Math.cos(lineAngle + headAngle) * headLength;
		double targetY = endY + Math.sin(lineAngle + headAngle) * headLength;

		shape.lineTo(targetX, targetY);
		shape.moveTo(endX, endY);

		targetX = endX + Math.cos(lineAngle - headAngle) * headLength;
		targetY = endY + Math.sin(lineAngle - headAngle) * headLength;

		shape.lineTo(targetX, targetY);

		shape.closePath();

		g2.draw(shape);
	}
}
