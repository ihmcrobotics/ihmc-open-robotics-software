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

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;

import sun.awt.image.IntegerInterleavedRaster;
import us.ihmc.sensorProcessing.bubo.gui.SpacialDisplay;
import us.ihmc.sensorProcessing.bubo.maps.d2.grid.GridMapSpacialInfo;
import us.ihmc.sensorProcessing.bubo.maps.d2.grid.OccupancyGrid2D_F32;

/**
 * Displays an OccupancyGrid2D_F32 as a 2D gray scale image.  Basic display that does not have any tools to interact
 * with the map.  Is designed to be able to display large and small maps efficiently by only rendering what is
 * visible and saving the results to be reused if nothing changes.
 *
 * @author Peter Abeles
 */
public class GridMapBasicDisplay extends SpacialDisplay {

	// how many pixels wide is the scale bar
	private final int SCALE_BAR_PIXELS = 50;
	// the map being displayed
	protected OccupancyGrid2D_F32 map;
	// spacial information on the map being shown
	protected GridMapSpacialInfo spacial;
	// the map's size in pixels the last time setComponentSize() was called
	// this is saved to avoid excessive calls to resize the component causing flickering
	int oldMapWidthPixels = -1;
	int oldMapHeightPixels = -1;
	// rendered image of the map that's the same size as the view port
	private BufferedImage rendered;
	// color used for out of bounds data
	private int colorOutOfBounds = 0x11 << 16 | 0x11 << 8 | 0x11;
	// color for cells whose statistics are not known yet
	private int colorUnknown = 0x11 << 16 | 0x11 << 8 | 0xFF;
	// Used to see if the view has changed and the map needs to be rendered again
	private Rectangle previousView = new Rectangle();
	// Is the map being forced to be rerendered
	private boolean forceRender;

	public GridMapBasicDisplay() {
		showCoordinateAxis("Y", true, "X", true);
	}

	/**
	 * Specifies what color unknown sections of the map should be.  If a null is provided then
	 * no special color will be used to indicate that its unknown.
	 *
	 * @param c
	 */
	public void setColorUnknown(Color c) {
		if (c == null)
			colorUnknown = -1;
		else
			colorUnknown = c.getRGB();
	}

	/**
	 * Changes or sets the map being displayed.
	 *
	 * @param spacial Spacial information on the map
	 * @param map     Map data.
	 */
	public void setMap(GridMapSpacialInfo spacial, OccupancyGrid2D_F32 map) {
		this.spacial = spacial;
		this.map = map;
		setComponentSize();

		// show a length that will be at least 50 pixels
		showScale(true, SCALE_BAR_PIXELS);
	}

	/**
	 * Updates GUI computes with the new map size
	 */
	private void setComponentSize() {
		if (map != null) {
			int mapWidthPixels = (int) Math.ceil(map.getWidth() * getPixelsPerCell());
			int mapHeightPixels = (int) Math.ceil(map.getHeight() * getPixelsPerCell());

			// if no change skip just in case GUI events are triggered
			// by just calling these functions
			if (oldMapWidthPixels == mapWidthPixels && oldMapHeightPixels == mapHeightPixels)
				return;

			oldMapWidthPixels = mapWidthPixels;
			oldMapHeightPixels = mapHeightPixels;

			setSize(mapWidthPixels, mapHeightPixels);
			setPreferredSize(getSize());
			setMinimumSize(getSize());
		}
	}

	/**
	 * The map has changed and needs to be redrawn on the next update.
	 */
	public void mapChanged() {
		setComponentSize();
		forceRender = true;
	}

	/**
	 * Changes the scale in which the map is displayed and updates all the related vision components.
	 *
	 * @param pixelsPerMeter Number of pixels in a meter is the display showing.
	 */
	@Override
	public void setPixelsPerMeter(double pixelsPerMeter) {
		super.setPixelsPerMeter(pixelsPerMeter);
		setComponentSize();
		forceRender = true;
	}


//	public void setMap(GridMapSpacialInfo spacial, OccupancyGrid2D_I map) {
//		setMap(spacial, new WrapOccupancy2D_I_to_F32(map));
//	}


	/**
	 * Draws the map into a BufferedImage that is exactly the size of the visible region.
	 *
	 * @param visible
	 */
	protected void renderMap(Rectangle visible) {
		if (map == null || spacial == null)
			return;

		int width = visible.width;
		int height = visible.height;

		// number of pixels in a map cell
		double cellPixels = getPixelsPerCell();

		int pixelOffX = (int) Math.floor(visible.x % cellPixels);
		int pixelOffY = (int) Math.floor(visible.y % cellPixels);

		int offsetMapX = (int) Math.floor(visible.x / cellPixels);
		int offsetMapY = (int) Math.floor(visible.y / cellPixels);

		boolean declareMap = rendered == null;

		if (!declareMap) {
			declareMap = rendered.getWidth() != width || rendered.getHeight() != height;
		}

		if (declareMap) {
			rendered = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		}

		if (rendered.getRaster() instanceof IntegerInterleavedRaster) {
			fastRender(offsetMapX, offsetMapY, pixelOffX, pixelOffY, cellPixels, width, height);
		} else {
			safeRender(offsetMapX, offsetMapY, pixelOffX, pixelOffY, cellPixels, width, height);
		}
	}

	/**
	 * Number of pixels per map cell
	 */
	private double getPixelsPerCell() {
		return spacial.getCellSize() * getPixelsPerMeter();
	}

	/**
	 * Writes directly to the raw data in the buffered image.
	 */
	private void fastRender(int offsetX, int offsetY, int pixelOffX, int pixelOffY, double cellPixels, int width, int height) {
		IntegerInterleavedRaster raster = (IntegerInterleavedRaster) rendered.getRaster();
		int[] imageData = raster.getDataStorage();
		int imageIndex = 0;

		double pixelsToCell = 1.0 / cellPixels;

		// step through all the pixels
		for (int i = height - 1; i >= 0; i--) {   // invert the axis so that +y is on top
			int y = offsetY + (int) ((i + pixelOffY) * pixelsToCell);

			for (int j = 0; j < width; j++, imageIndex++) {
				int x = offsetX + (int) ((j + pixelOffX) * pixelsToCell);

				if (map.isInBounds(x, y)) {
					if (colorUnknown < 0 || map.isKnown(x, y)) {
						int val = (int) (map.get(x, y) * 255.0f);
						imageData[imageIndex] = val << 16 | val << 8 | val;
					} else {
						imageData[imageIndex] = colorUnknown;
					}
				} else {
					imageData[imageIndex] = colorOutOfBounds;
				}
			}
		}
		// lets it know it has been modified
		rendered.setRGB(0,0,rendered.getRGB(0,0));
	}

	/**
	 * Writes to the buffered image using its RGB interface
	 */
	private void safeRender(int offsetX, int offsetY, int pixelOffX, int pixelOffY, double cellPixels, int width, int height) {
		double pixelsToCell = 1.0 / cellPixels;

		// step through all the pixels
		for (int i = height - 1; i >= 0; i--) {  // invert the axis so that +y is on top
			// grid cell y-coordinate
			int y = offsetY + (int) ((i + pixelOffY) * pixelsToCell);

			for (int j = 0; j < width; j++) {
				// grid cell x-coordinate
				int x = offsetX + (int) ((j + pixelOffX) * pixelsToCell);

				if (map.isInBounds(x, y)) {
					if (colorUnknown < 0 || map.isKnown(x, y)) {
						int val = (int) (map.get(x, y) * 255);
						rendered.setRGB(j, i, val << 16 | val << 8 | val);
					} else {
						rendered.setRGB(j, i, colorUnknown);
					}
				} else {
					rendered.setRGB(j, i, colorOutOfBounds);
				}
			}
		}
	}

	/**
	 * Draws the only rendered map or renders it again if the view has changed.
	 *
	 * @param g
	 */
	@Override
	public void paintComponent(Graphics g) {
		Rectangle r = getVisibleRect();
//        System.out.println("Visible = "+r.x+" "+r.y+" "+r.width+" "+r.height+"  component size "+getWidth()+" "+getHeight());

		Graphics2D g2 = (Graphics2D) g;

		if (map == null)
			return;

		// only render if the view has changed
		if (forceRender || r.x != previousView.x ||
				r.y != previousView.y ||
				r.width != previousView.width ||
				r.height != previousView.height) {
			forceRender = false;
			renderMap(getVisibleMapRect());
			previousView.setBounds(r);
		}

		g2.drawImage(rendered, r.x, r.y, null);
	}

	/**
	 * Returns a rectangle containing the area of the map being viewed in the view.
	 *
	 * @return rectangle of the map being viewed in pixels.
	 */
	protected Rectangle getVisibleMapRect() {
		Rectangle r = getVisibleRect();
		r.y = getHeight() - r.y - r.height;

		return r;
	}


	/**
	 * Converts a x-axis coordinate in the map's reference frame into a x-axis coordinate in the GUI component's
	 * reference frame.
	 *
	 * @param mapX x-coordinate in map
	 * @return x-coordinate in GUI
	 */
	public double mapToGuiX(double mapX) {
		return mapX * getPixelsPerMeter();
	}

	/**
	 * Converts a y-axis coordinate in the map's reference frame into a y-axis coordinate in the GUI component's
	 * reference frame.
	 *
	 * @param mapY y-coordinate in map
	 * @return y-coordinate in GUI
	 */
	public double mapToGuiY(double mapY) {
		return getHeight() - mapY * getPixelsPerMeter();
	}

//    public static void main( String args[] ) {
//        String fileName = "/home/pja/prescription.jpg";
//        BufferedImage image = UtilImages.loadImage(fileName);
//        OccupancyGrid2D_F32 map = OccupancyGridIO.load_F32(image);
//        GridMapSpacialInfo spacial = new GridMapSpacialInfo(0.1,null);
//
//        GridMapBasicDisplay comp = new GridMapBasicDisplay();
//        comp.setMap(spacial,map);
//
//        UtilDisplayBubo.show(comp,"OccupancyGrid",true,0,0,200,200);
//    }
}
