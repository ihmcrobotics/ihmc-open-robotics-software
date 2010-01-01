package us.ihmc.plotting.shapes;

import java.awt.*;
import java.io.*;

import javax.vecmath.*;

import us.ihmc.plotting.*;
import java.util.ArrayList;

public class PolyLine
    extends Artifact implements Serializable
{
    private ArrayList<Point2d> points;
    private int lineThickness = 1;

    public PolyLine(String id)
    {
        super(id);
    }

    public PolyLine(String id, ArrayList<Point2d> points)
    {
        super(id);
        this.points = points;
    }

    public void setPoints(ArrayList<Point2d> points)
    {
        this.points = points;
    }

    public void setLineThicknessInPixels(int pixels)
    {
        this.lineThickness = pixels;
    }


    /**
     * Must provide a draw method for plotter to render artifact
     */
    public void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
    {
        if(isVisible)
        {
            for(int i = 0; i < points.size(); i++)
            {
                int x1 = Xcenter + (new Double(points.get(i-1).x * scaleFactor).intValue());
                int y1 = Ycenter - (new Double(points.get(i-1).y * scaleFactor).intValue());
                int x2 = Xcenter + (new Double(points.get(i).x * scaleFactor).intValue());
                int y2 = Ycenter - (new Double(points.get(i).y * scaleFactor).intValue());
                g.setColor(color);
                Graphics2D g2d = (Graphics2D) g;
                Stroke currentStroke = g2d.getStroke();
                g2d.setStroke(new BasicStroke(lineThickness));
                g.drawLine(x1, y1, x2, y2);
                g2d.setStroke(currentStroke);
            }
        }
    }
    public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
    {
    }
}


