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

package us.ihmc.sensorProcessing.bubo.maps.d2.grid;

import georegression.struct.point.Point2D_F64;

/**
 * Description of the grid map's spacial information and its location in the global frame.  All occupancy
 * grids are assumed to be aligned along the global coordinate system's axis.  This makes look up much easier
 * since a rotation does not need to be performed.  All cells have a constant size.
 *
 * @author Peter Abeles
 */
public class GridMapSpacialInfo {

	// size of a grid cell in global units
	double cellSize;

	// bottom left corner of the map in the global frame
	Point2D_F64 bl;

	/**
	 * @param cellSize   Size of a map cell
	 * @param bottomLeft Bottom left coordinate of the map
	 */
	public GridMapSpacialInfo(double cellSize, Point2D_F64 bottomLeft) {
		this.cellSize = cellSize;
		this.bl = bottomLeft.copy();
	}

	/**
	 * @param cellSize Size of a map cell
	 * @param bl_x     Bottom left of the map.  x-coordinate
	 * @param bl_y     Bottom left of the map.  y-coordinate
	 */
	public GridMapSpacialInfo(double cellSize, double bl_x, double bl_y) {
		this.cellSize = cellSize;
		this.bl = new Point2D_F64(bl_x, bl_y);
	}

	public GridMapSpacialInfo() {
	}

	/**
	 * Convert from global coordinates into map cell coordinates.
	 */
	public void globalToMap(Point2D_F64 global, Point2D_F64 map) {
		map.x = (global.x - bl.x) / cellSize;
		map.y = (global.y - bl.y) / cellSize;
	}

	/**
	 * Convert from map cell coordinates into global coordinates
	 */
	public void mapToGlobal(Point2D_F64 map, Point2D_F64 global) {
		global.x = map.x * cellSize + bl.x;
		global.y = map.y * cellSize + bl.y;
	}

	public double getCellSize() {
		return cellSize;
	}

	public Point2D_F64 getBl() {
		return bl;
	}
}
