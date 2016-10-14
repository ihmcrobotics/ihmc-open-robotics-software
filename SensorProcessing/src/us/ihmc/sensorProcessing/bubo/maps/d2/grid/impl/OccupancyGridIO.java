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

package us.ihmc.sensorProcessing.bubo.maps.d2.grid.impl;

import java.awt.image.BufferedImage;

import us.ihmc.sensorProcessing.bubo.maps.d2.grid.OccupancyGrid2D_F32;

/**
 * @author Peter Abeles
 */
// TODO move to Bubo IO?
public class OccupancyGridIO {

	/**
	 * Converts an image into a ArrayGrid2D_F32
	 *
	 * @param image Input image which is being converted
	 * @return The new map.
	 */
	public static ArrayGrid2D_F32 load_F32(BufferedImage image) {
		int width = image.getWidth();
		int height = image.getHeight();

		ArrayGrid2D_F32 map = new ArrayGrid2D_F32(width, height);

		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int rgb = image.getRGB(j, i);

				int val = (rgb >> 16) & 0xFF;
				val += (rgb >> 8) & 0xFF;
				val += rgb & 0xFF;

				map.set(j, i, val / (255.0f * 3.0f));
			}
		}

		return map;
	}


	/**
	 * Converts OccupancyGrid2D_F32 into an image
	 *
	 * @param map         Occupancy grid
	 * @param invertColor switch between walls being white or dark
	 * @return An image of the map.
	 */
	public static BufferedImage render_F32(OccupancyGrid2D_F32 map, boolean invertColor) {
		int width = map.getWidth();
		int height = map.getHeight();

		BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);

		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int val = (int) (map.get(x, y) * 255.0f);
				if (invertColor)
					val = 255 - val;
				int rgb = val << 16 | val << 8 | val;

				image.setRGB(x, y, rgb);
			}
		}

		return image;
	}
}
