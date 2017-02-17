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

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JScrollPane;
import javax.swing.JToolBar;

/**
 * <p>
 * A scrollable component designed for viewing SpacialDisplays.  The SpacialDisplay is viewed inside a JScrollPane
 * and can be dragged around with a mouse or moved with key commands.  A toolbar is provided for zooming in and out.
 * </p>
 * <p/>
 * <p>
 * The object which is being viewed is either passed in to the constructor or through {@link #setDisplay(SpacialDisplay)}.
 * </p>
 *
 * @author Peter Abeles
 */
public class ScrollableSpacialPane extends JComponent implements ActionListener, MouseListener, MouseMotionListener {

	// the scroll pane containing the display
	protected JScrollPane scrollPane;
	// reference to the object being displayed
	protected SpacialDisplay display;
	// the tool bar used to display buttons
	protected JToolBar toolbar;
	// buttons used to change the zoom factor
	private JButton bReset, bIn, bOut;
	// when home is processed what the pixels per meter is set to
	private double homePixelPerMeters;
	// last location of the mouse while dragging
	private int pressedX;
	private int pressedY;
	// the minimum number of pixels per meter allowed
	private double minimumPixelsPerMeter = 0.0001;


	public ScrollableSpacialPane() {

		setLayout(new BorderLayout());
		addToolbar();

		scrollPane = new JScrollPane();
		add(scrollPane, BorderLayout.CENTER);

		setPreferredSize(new Dimension(200, 200));
	}

	public ScrollableSpacialPane(SpacialDisplay display) {
		this();
		setDisplay(display);
	}

	public void setMinimumPixelsPerMeter(double minimumPixelsPerMeter) {
		this.minimumPixelsPerMeter = minimumPixelsPerMeter;
	}

	/**
	 * Specifies the SpacialDisplay object that is to be viewed.
	 *
	 * @param display The SpacialDisplay being viewed.
	 */
	public void setDisplay(SpacialDisplay display) {
		if (this.display != null) {
			this.display.removeMouseListener(this);
			this.display.removeMouseMotionListener(this);
			scrollPane.removeAll();
		}
		this.display = display;
		this.homePixelPerMeters = display.getPixelsPerMeter();

		display.addMouseListener(this);
		display.addMouseMotionListener(this);

		scrollPane.getViewport().setView(display);
	}

	/**
	 * Makes the specified coordinates be the center of the view in GUI coordinates.
	 *
	 * @param x x-coordinate in GUI pixels
	 * @param y y-coordinate in GUI pixels
	 */
	public void setViewCenter(int x, int y) {
		Rectangle r = getVisibleRect();

		scrollPane.getVerticalScrollBar().setValue(y - (int) r.getHeight() / 2);
		scrollPane.getHorizontalScrollBar().setValue(x - (int) r.getWidth() / 2);
	}

	private void addToolbar() {
		toolbar = new JToolBar("Grid Map Tools");

		bReset = new JButton("Reset");
		bReset.addActionListener(this);
		bIn = new JButton("In");
		bIn.addActionListener(this);
		bOut = new JButton("Out");
		bOut.addActionListener(this);

		toolbar.add(bReset);
		toolbar.add(bIn);
		toolbar.add(bOut);

		add(toolbar, BorderLayout.NORTH);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == bReset) {
			adjustPixelsPerMeter(homePixelPerMeters / display.getPixelsPerMeter());
			repaint();
		} else if (e.getSource() == bIn) {
			adjustPixelsPerMeter(1.25);
			repaint();
		} else if (e.getSource() == bOut) {
			adjustPixelsPerMeter(0.75);

			repaint();
		}
	}

	private void adjustPixelsPerMeter(double scaleFactor) {
		// get the current view center
		Rectangle r = getVisibleRect();
		int c_x = scrollPane.getHorizontalScrollBar().getValue() + (int) r.getWidth() / 2;
		int c_y = scrollPane.getVerticalScrollBar().getValue() + (int) r.getHeight() / 2;

		double scale = display.getPixelsPerMeter();
		double oldScale = scale;
		scale *= scaleFactor;
		if (scale <= minimumPixelsPerMeter)
			scale = minimumPixelsPerMeter;
		display.setPixelsPerMeter(scale);
		scrollPane.getViewport().setView(display);

		// adjust for the change in scale
		c_x *= scale / oldScale;
		c_y *= scale / oldScale;

		setViewCenter(c_x, c_y);
	}

	@Override
	public void mouseClicked(MouseEvent e) {
		display.requestFocus();
	}

	@Override
	public void mousePressed(MouseEvent e) {
		display.requestFocus();
		pressedX = e.getXOnScreen();
		pressedY = e.getYOnScreen();
	}

	@Override
	public void mouseReleased(MouseEvent e) {
	}

	@Override
	public void mouseEntered(MouseEvent e) {
	}

	@Override
	public void mouseExited(MouseEvent e) {
	}

	@Override
	public void mouseDragged(MouseEvent e) {

		int deltaX = e.getXOnScreen() - pressedX;
		int deltaY = e.getYOnScreen() - pressedY;
		pressedX = e.getXOnScreen();
		pressedY = e.getYOnScreen();

		int valX = scrollPane.getHorizontalScrollBar().getValue();
		int valY = scrollPane.getVerticalScrollBar().getValue();

//        System.out.println("Max horiz = "+scrollPane.getHorizontalScrollBar().getMaximum()+" val "+valX+" deltaX "+deltaX+" pressedX "+pressedX);

		scrollPane.getHorizontalScrollBar().setValue(valX - deltaX);
		scrollPane.getVerticalScrollBar().setValue(valY - deltaY);
	}

	@Override
	public void mouseMoved(MouseEvent e) {
	}

}
