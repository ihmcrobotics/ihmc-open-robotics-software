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

package us.ihmc.sensorProcessing.bubo.gui.maps;

import java.awt.Dimension;

import us.ihmc.sensorProcessing.bubo.gui.ScrollableSpacialPane;
import us.ihmc.sensorProcessing.bubo.maps.d2.grid.GridMapSpacialInfo;
import us.ihmc.sensorProcessing.bubo.maps.d2.grid.OccupancyGrid2D_F32;

/**
 * A display component for OccupancyGrid2D_F32 that allows the user to scroll around, zoom in and out, and displays
 * information about the map.
 *
 * @author Peter Abeles
 */
// todo information in bar: cell size, mouse grid coordinate
// todo comment
public class GridMapInteractionDisplay extends ScrollableSpacialPane {

	GridMapBasicDisplay mapDisplay;

	public GridMapInteractionDisplay() {
		mapDisplay = new GridMapBasicDisplay();
		setDisplay(mapDisplay);
	}

	public GridMapInteractionDisplay(GridMapBasicDisplay mapDisplay) {
		this.mapDisplay = mapDisplay;
		setDisplay(mapDisplay);
	}

	public void setMap(GridMapSpacialInfo spacial, OccupancyGrid2D_F32 map) {
		mapDisplay.setMap(spacial, map);
	}

	public GridMapBasicDisplay getMapDisplay() {
		return mapDisplay;
	}

	/**
	 * Updates the preferred, minimum, and maximum size using the mapDisplay
	 */
	public void resizeToFitMap() {
		// there is bound to be a better way to do this.  Solutions I found searching online didn't seem to work
		Dimension dmap = mapDisplay.getPreferredSize();
		Dimension dtool = toolbar.getPreferredSize();
		setPreferredSize(new Dimension(dmap.width + 5, dtool.height + dmap.height + 5));
		setMinimumSize(getPreferredSize());
		setMaximumSize(getPreferredSize());
		revalidate();
	}
}
