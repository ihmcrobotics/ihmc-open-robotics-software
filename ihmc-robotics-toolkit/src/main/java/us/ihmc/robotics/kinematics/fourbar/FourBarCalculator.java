package us.ihmc.robotics.kinematics.fourbar;

import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FourBarCalculator
{
   /*
    * @formatter:off
    *  Representation of the four bar with name correspondences:
    *        DA
    *    D--------A
    *    |\      /|
    *    | \DB  / |
    *    |  \  /  |
    * CD |   \/   | AB
    *    |   /\   |
    *    |  /  \  |
    *    | /AC  \ |
    *    |/      \|
    *    C--------B
    *        BC
    * Angle name convention:
    * - Inner angle at vertex A: DAB
    * - Inner angle at vertex B: ABC
    * - Inner angle at vertex C: BCD
    * - Inner angle at vertex D: CDA
    * @formatter:on
    */

   private final Vertex A = new Vertex("A");
   private final Vertex B = new Vertex("B");
   private final Vertex C = new Vertex("C");
   private final Vertex D = new Vertex("D");

   private final Edge AB = new Edge("AB");
   private final Edge BC = new Edge("BC");
   private final Edge CD = new Edge("CD");
   private final Edge DA = new Edge("DA");

   private final Diagonal AC = new Diagonal("AC");
   private final Diagonal BD = new Diagonal("AC");

   public enum Angle
   {
      ABC, BCD, CDA, DAB
   };

   public FourBarCalculator()
   {
      A.setup(DA, AB, AC);
      B.setup(AB, BC, BD);
      C.setup(BC, CD, AC);
      D.setup(CD, DA, BD);

      AB.setup(A, B, DA, BC);
      BC.setup(B, C, AB, CD);
      CD.setup(C, D, BC, DA);
      DA.setup(D, A, CD, AB);

      AC.setup(A, C, BD);
      BD.setup(B, D, AC);
   }

   public void setup(Point2DReadOnly A, Point2DReadOnly B, Point2DReadOnly C, Point2DReadOnly D)
   {
      setSideLengths(A.distance(B), B.distance(C), C.distance(D), D.distance(A));

   }

   public void setToNaN()
   {
      A.setToNaN();
      B.setToNaN();
      C.setToNaN();
      D.setToNaN();

      AB.setToNaN();
      BC.setToNaN();
      CD.setToNaN();
      DA.setToNaN();

      AC.setToNaN();
      BD.setToNaN();
   }

   public void setSideLengths(double AB, double BC, double CD, double DA)
   {
      setToNaN();
      this.AB.length = AB;
      this.BC.length = BC;
      this.CD.length = CD;
      this.DA.length = DA;
   }

   /**
    * <p>
    * Updates internally: all the angles and diagonal lengths.
    * </p>
    * 
    * @param source
    * @param angle
    * @return
    */
   public Bound update(Angle source, double angle)
   {
      switch (source)
      {
         case DAB:
            return FourbarCalculatorTools.update(getVertexA(), angle);
         case ABC:
            return FourbarCalculatorTools.update(getVertexB(), angle);
         case BCD:
            return FourbarCalculatorTools.update(getVertexC(), angle);
         case CDA:
            return FourbarCalculatorTools.update(getVertexD(), angle);
         default:
            throw new IllegalStateException();
      }
   }

   public Bound update(Angle source, double angle, double angleDot)
   {
      switch (source)
      {
         case DAB:
            return FourbarCalculatorTools.update(getVertexA(), angle, angleDot);
         case ABC:
            return FourbarCalculatorTools.update(getVertexB(), angle, angleDot);
         case BCD:
            return FourbarCalculatorTools.update(getVertexC(), angle, angleDot);
         case CDA:
            return FourbarCalculatorTools.update(getVertexD(), angle, angleDot);
         default:
            throw new IllegalStateException();
      }
   }

   public Bound update(Angle source, double angle, double angleDot, double angleDDot)
   {
      switch (source)
      {
         case DAB:
            return FourbarCalculatorTools.update(getVertexA(), angle, angleDot, angleDDot);
         case ABC:
            return FourbarCalculatorTools.update(getVertexB(), angle, angleDot, angleDDot);
         case BCD:
            return FourbarCalculatorTools.update(getVertexC(), angle, angleDot, angleDDot);
         case CDA:
            return FourbarCalculatorTools.update(getVertexD(), angle, angleDot, angleDDot);
         default:
            throw new IllegalStateException();
      }
   }

   public Vertex getVertexA()
   {
      return A;
   }

   public Vertex getVertexB()
   {
      return B;
   }

   public Vertex getVertexC()
   {
      return C;
   }

   public Vertex getVertexD()
   {
      return D;
   }

   public Edge getEdgeAB()
   {
      return AB;
   }

   public Edge getEdgeBC()
   {
      return BC;
   }

   public Edge getEdgeCD()
   {
      return CD;
   }

   public Edge getEdgeDA()
   {
      return DA;
   }

   public Diagonal getDiagonalAC()
   {
      return AC;
   }

   public Diagonal getDiagonalBD()
   {
      return BD;
   }

   public double getAngleDAB()
   {
      return getVertexA().getAngle();
   }

   public double getAngleABC()
   {
      return getVertexB().getAngle();
   }

   public double getAngleBCD()
   {
      return getVertexC().getAngle();
   }

   public double getAngleCDA()
   {
      return getVertexD().getAngle();
   }

   public double getAngleDtDAB()
   {
      return getVertexA().getAngleDot();
   }

   public double getAngleDtABC()
   {
      return getVertexB().getAngleDot();
   }

   public double getAngleDtBCD()
   {
      return getVertexC().getAngleDot();
   }

   public double getAngleDtCDA()
   {
      return getVertexD().getAngleDot();
   }

   public double getAngleDt2DAB()
   {
      return getVertexA().getAngleDDot();
   }

   public double getAngleDt2ABC()
   {
      return getVertexB().getAngleDDot();
   }

   public double getAngleDt2BCD()
   {
      return getVertexC().getAngleDDot();
   }

   public double getAngleDt2CDA()
   {
      return getVertexD().getAngleDDot();
   }

   public double getMinDAB()
   {
      return getVertexA().getMinAngle();
   }

   public double getMaxDAB()
   {
      return getVertexA().getMaxAngle();
   }

   public double getMinABC()
   {
      return getVertexB().getMinAngle();
   }

   public double getMaxABC()
   {
      return getVertexB().getMaxAngle();
   }

   public double getMinBCD()
   {
      return getVertexC().getMinAngle();
   }

   public double getMaxBCD()
   {
      return getVertexC().getMaxAngle();
   }

   public double getMinCDA()
   {
      return getVertexD().getMinAngle();
   }

   public double getMaxCDA()
   {
      return getVertexD().getMaxAngle();
   }

   public double getAB()
   {
      return getEdgeAB().getLength();
   }

   public double getBC()
   {
      return getEdgeBC().getLength();
   }

   public double getCD()
   {
      return getEdgeCD().getLength();
   }

   public double getDA()
   {
      return getEdgeDA().getLength();
   }

   public static class Vertex
   {
      private final String name;

      private double angle;
      private double angleDot;
      private double angleDDot;

      private double minAngle;
      private double maxAngle;

      private Edge nextEdge, previousEdge;
      private Diagonal diagonal;

      private Vertex(String name)
      {
         this.name = name;
      }

      private void setup(Edge previousEdge, Edge nextEdge, Diagonal diagonal)
      {
         this.nextEdge = nextEdge;
         this.previousEdge = previousEdge;
         this.diagonal = diagonal;
      }

      public void setToNaN()
      {
         angle = Double.NaN;
         angleDot = Double.NaN;
         angleDDot = Double.NaN;
         minAngle = Double.NaN;
         maxAngle = Double.NaN;
      }

      public void setToMin()
      {
         angle = getMinAngle();
         angle = 0.0;
         angleDDot = 0.0;
      }

      public void setToMax()
      {
         angle = getMaxAngle();
         angle = 0.0;
         angleDDot = 0.0;
      }

      public void setAngle(double angle)
      {
         this.angle = angle;
      }

      public void setAngleDot(double angleDot)
      {
         this.angleDot = angleDot;
      }

      public void setAngleDDot(double angleDDot)
      {
         this.angleDDot = angleDDot;
      }

      public void setMinAngle(double minAngle)
      {
         this.minAngle = minAngle;
      }

      public void setMaxAngle(double maxAngle)
      {
         this.maxAngle = maxAngle;
      }

      public String getName()
      {
         return name;
      }

      public double getAngle()
      {
         return angle;
      }

      public double getAngleDot()
      {
         return angleDot;
      }

      public double getAngleDDot()
      {
         return angleDDot;
      }

      public double getMinAngle()
      {
         if (Double.isNaN(minAngle))
            FourbarCalculatorTools.updateMinAngle(this);
         return minAngle;
      }

      public double getMaxAngle()
      {
         if (Double.isNaN(maxAngle))
            FourbarCalculatorTools.updateMaxAngle(this);
         return maxAngle;
      }

      public Edge getNextEdge()
      {
         return nextEdge;
      }

      public Edge getPreviousEdge()
      {
         return previousEdge;
      }

      public Diagonal getDiagonal()
      {
         return diagonal;
      }

      public Vertex getNextVertex()
      {
         return nextEdge.getEnd();
      }

      public Vertex getPreviousVertex()
      {
         return previousEdge.getStart();
      }

      public Vertex getOppositeVertex()
      {
         return nextEdge.getNext().getEnd();
      }

      @Override
      public String toString()
      {
         return getName();
      }
   }

   public static class Edge
   {
      private final String name;

      private double length;
      private Vertex start, end;
      private Edge nextEdge, previous;

      private Edge(String name)
      {
         this.name = name;
      }

      private void setup(Vertex start, Vertex end, Edge previous, Edge next)
      {
         this.start = start;
         this.end = end;
         this.previous = previous;
         this.nextEdge = next;
      }

      public void setToNaN()
      {
         length = Double.NaN;
      }

      public String getName()
      {
         return name;
      }

      public double getLength()
      {
         return length;
      }

      public Vertex getStart()
      {
         return start;
      }

      public Vertex getEnd()
      {
         return end;
      }

      public Edge getNext()
      {
         return nextEdge;
      }

      public Edge getPrevious()
      {
         return previous;
      }

      @Override
      public String toString()
      {
         return getName();
      }
   }

   public static class Diagonal
   {
      private final String name;

      private double length;
      private double lengthDot;
      private double lengthDDot;
      private double maxLength;
      private Vertex start, end;
      private Diagonal other;

      public Diagonal(String name)
      {
         this.name = name;
      }

      private void setup(Vertex start, Vertex end, Diagonal other)
      {
         this.start = start;
         this.end = end;
         this.other = other;
      }

      public void setToNaN()
      {
         length = Double.NaN;
         lengthDot = Double.NaN;
         lengthDDot = Double.NaN;
         maxLength = Double.NaN;
      }

      public void setLength(double length)
      {
         this.length = length;
      }

      public void setLengthDot(double lengthDot)
      {
         this.lengthDot = lengthDot;
      }

      public void setLengthDDot(double lengthDDot)
      {
         this.lengthDDot = lengthDDot;
      }

      public void setMaxLength(double maxLength)
      {
         this.maxLength = maxLength;
      }

      public String getName()
      {
         return name;
      }

      public double getLength()
      {
         return length;
      }

      public double getLengthDot()
      {
         return lengthDot;
      }

      public double getLengthDDot()
      {
         return lengthDDot;
      }

      public double getMaxLength()
      {
         if (Double.isNaN(maxLength))
            FourbarCalculatorTools.updateDiagonalMaxLength(this);
         return maxLength;
      }

      public Vertex getStart()
      {
         return start;
      }

      public Vertex getEnd()
      {
         return end;
      }

      public Diagonal getOther()
      {
         return other;
      }

      @Override
      public String toString()
      {
         return getName();
      }
   }
}
