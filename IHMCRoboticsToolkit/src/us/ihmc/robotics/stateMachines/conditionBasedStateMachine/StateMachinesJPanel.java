package us.ihmc.robotics.stateMachines.conditionBasedStateMachine;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Map;

import javax.swing.JPanel;

import org.jgraph.JGraph;
import org.jgraph.graph.DefaultCellViewFactory;
import org.jgraph.graph.DefaultEdge;
import org.jgraph.graph.DefaultGraphCell;
import org.jgraph.graph.DefaultGraphModel;
import org.jgraph.graph.GraphConstants;
import org.jgraph.graph.GraphLayoutCache;

import com.jgraph.layout.JGraphFacade;
import com.jgraph.layout.tree.JGraphTreeLayout;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

/**
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2000</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class StateMachinesJPanel<E extends Enum<E>> extends JPanel implements StateChangedListener<E>
{
   private static final long serialVersionUID = 2453853798153829891L;
   private final StateMachine<E> stateMachine;
   private boolean oldStateDiagram;
   private DefaultGraphCell[] stateCells;
   private JGraph graph;
   private int numberOfStates;
   private Enum<?> currentState = null;


   public StateMachinesJPanel(StateMachine<E> stateMachine)
   {
      this(stateMachine, false);

//    System.out.println("state machine window created");
   }

   public StateMachinesJPanel(final StateMachine<E> stateMachine, boolean OldStateDiagram)
   {
      this.stateMachine = stateMachine;
      this.numberOfStates = stateMachine.states.size();
      this.oldStateDiagram = OldStateDiagram;
      this.currentState = stateMachine.getStateYoVariable().getEnumValue();

      if (!OldStateDiagram)
      {
         graph = new JGraph();
         initializeJPanel();
      }


      Thread repaintWhenStateChange = new Thread(new Runnable()
      {
         public void run()
         {
            while (true)
            {
               // System.out.println("painting");
               try
               {
                  Thread.sleep(100);
               }
               catch (InterruptedException e)
               {
                  e.printStackTrace();
               }

               if (!currentState.name().equals(stateMachine.getStateYoVariable().getEnumValue().name()))
               {
                  currentState = stateMachine.getStateYoVariable().getEnumValue();
                  updateStateMachine();
               }
            }
         }
      });
      repaintWhenStateChange.start();

   }

   public void updateStateMachine()
   {
      State<E> currentState = stateMachine.getCurrentState();
      if (oldStateDiagram)
      {
         this.repaint();
      }
      else
      {
         int currentIndex = indexOfStateInStateCells(currentState);
         GraphConstants.setBackground(graph.getAttributes(stateCells[currentIndex]), Color.RED);
         stateCells[currentIndex].setAttributes(graph.getAttributes(stateCells[currentIndex]));

         for (int i = 0; i < stateCells.length; i++)
         {
            if (i != currentIndex)
            {
               GraphConstants.setBackground(graph.getAttributes(stateCells[i]), Color.CYAN);
               stateCells[i].setAttributes(graph.getAttributes(stateCells[i]));
            }
         }

         graph.repaint();
         graph.refresh();

      }
   }

   public void stateChanged(State<E> oldState, State<E> newState, double time)
   {
      if (oldStateDiagram)
      {
         this.repaint();
      }
      else
      {
         int newIndex = indexOfStateInStateCells(newState);
         int oldIndex = indexOfStateInStateCells(oldState);

         GraphConstants.setBackground(graph.getAttributes(stateCells[oldIndex]), Color.CYAN);
         GraphConstants.setBackground(graph.getAttributes(stateCells[newIndex]), Color.RED);

         stateCells[oldIndex].setAttributes(graph.getAttributes(stateCells[oldIndex]));
         stateCells[newIndex].setAttributes(graph.getAttributes(stateCells[newIndex]));

         graph.repaint();
         this.repaint();
         graph.refresh();
      }
   }

   private int[] sourceArrows = new int[100];
   private int[] targetArrows = new int[100];
   private int sourceArrowIndex = 0;
   private int targetArrowIndex = 0;

   private void createArrow(int source, int target)
   {
      if (!arrowAlreadyExist(source, target))
      {
         DefaultGraphCell[] arrowCell = new DefaultGraphCell[1];
         DefaultEdge edge = new DefaultEdge();
         edge.setSource(stateCells[source].getChildAt(0));
         edge.setTarget(stateCells[target].getChildAt(0));
         arrowCell[0] = edge;
         int arrow = GraphConstants.ARROW_TECHNICAL;
         GraphConstants.setLineEnd(edge.getAttributes(), arrow);
         GraphConstants.setEndFill(edge.getAttributes(), true);
         graph.getGraphLayoutCache().insert(arrowCell);

         sourceArrows[sourceArrowIndex] = source;
         sourceArrowIndex++;
         targetArrows[targetArrowIndex] = target;
         targetArrowIndex++;
      }
   }


   private boolean arrowAlreadyExist(int source, int target)
   {
      for (int i = 0; i < sourceArrows.length; i++)
      {
         if ((sourceArrows[i] == source) && (targetArrows[i] == target))
         {
            return true;
         }
      }

      return false;
   }

   private DefaultGraphCell createCell(String name, Point2D placement, int index)
   {
//      System.out.println(name + " " + placement + " " + index);
      stateCells[index] = new DefaultGraphCell(new String(name));
      Font f = new Font("Arial", Font.PLAIN, 12);
      GraphConstants.setFont(stateCells[index].getAttributes(), f);
      GraphConstants.setAutoSize(stateCells[index].getAttributes(), true);
      GraphConstants.setOpaque(stateCells[index].getAttributes(), true);

      Color color = colorStateCell(stateMachine.states.get(indexOfStateinStateMachine(name)));
      GraphConstants.setBackground(stateCells[index].getAttributes(), color);
      GraphConstants.setBounds(stateCells[index].getAttributes(), new Rectangle2D.Double(placement.getX(), placement.getY(), 0, 0));
      stateCells[index].addPort();

      return stateCells[index];
   }

   private int indexOfStateinStateMachine(String state)
   {
      for (int i = 0; i < stateMachine.states.size(); i++)
      {
         if (stateMachine.states.get(i).getStateEnum().toString().equals(state))
         {
            return i;
         }
      }

      return -1;
   }

   private int indexOfStateInStateCells(State<E> state)
   {
      for (int i = 0; i < stateCells.length; i++)
      {
         if (stateCells[i].toString().equals(state.getStateEnum().toString()))
         {
            return i;
         }
      }

      return -1;
   }

   private State<E> getParentState()
   {
      State<E> parentState = null;
      int max = 0;
      boolean needParentForDiagram = false;
      for (int i = 0; i < numberOfStates; i++)
      {
         State<E> state = stateMachine.states.get(i);
         if (state.getStateTransitions().size() == max)
         {
            needParentForDiagram = true;
         }

         if (state.getStateTransitions().size() > max)
         {
            max = state.getStateTransitions().size();
            needParentForDiagram = false;
            parentState = state;
         }
      }

      if (needParentForDiagram)
      {
         parentState = stateMachine.states.get(0);
      }

      return parentState;
   }

   private boolean isStateEmpty(State<E> checkstate)
   {
      boolean isEmpty = true;

      // check each state
      for (int i = 0; i < stateMachine.states.size(); i++)
      {
         State<E> state = stateMachine.states.get(i);

         // if it has a default state
         StateTransition<E> defaultNextStateTransition = state.getDefaultNextStateTransition();
         if (defaultNextStateTransition != null)
         {
            // and that default state if the one we are checking, then the one we are checking is not empty.
            if ((defaultNextStateTransition.getNextStateEnum() == checkstate.getStateEnum()))
            {
               isEmpty = false;
            }
         }

         // if this state has other state transitions
         for (int j = 0; j < state.getStateTransitions().size(); j++)
         {
            // and one of those state transitions are the state we are checking,
            // then the state we are checking is not empty
            if (state.getStateTransitions().get(j).getNextStateEnum() == checkstate.getStateEnum())
            {
               isEmpty = false;
            }

         }
      }

      // is empty if there are no states going to the check state.
      return isEmpty;
   }

   private boolean doesCellAlreadyExist(String state)
   {
      for (int i = 0; i < stateCells.length; i++)
      {
         if (stateCells[i] != null)
         {
            if (stateCells[i].toString().equals(state))
            {
               return true;
            }
         }
      }

      return false;
   }

   private int stateCellIndex = 0;

   private void createStateMachineDiagram(State<E> parent)
   {
      // if (!isStateEmpty(parent))
      {
         for (int i = 0; i < parent.getStateTransitions().size(); i++)
         {
            String state = parent.getStateTransitions().get(i).getNextStateEnum().toString();

            if (!doesCellAlreadyExist(state))
            {
               stateCells[stateCellIndex] = createCell(state, new Point2D(0, 0), stateCellIndex);
               createArrow(indexOfStateInStateCells(parent), stateCellIndex);
               stateCellIndex++;

               if (stateMachine.states.get(indexOfStateinStateMachine(state)).getStateTransitions().size() != 0)
               {
                  createStateMachineDiagram(stateMachine.states.get(indexOfStateinStateMachine(state)));
               }
               else
               {
                  try
                  {
                     for (int j = indexOfStateinStateMachine(state) + 1;
                             j < indexOfStateinStateMachine(parent.getStateTransitions().get(i + 1).getNextStateEnum().toString()); j++)
                     {
                        State<E> checkState = stateMachine.states.get(j);

                        if (!doesCellAlreadyExist(checkState.getStateEnum().toString()))
                        {
                           stateCells[stateCellIndex] = createCell(checkState.getStateEnum().toString(), new Point2D(0, 0), stateCellIndex);
                           createArrow(stateCellIndex - 1, stateCellIndex);
                           stateCellIndex++;
                        }
                     }
                  }
                  catch (Exception noAdditionalStateTransitions)
                  {
                     for (int j = indexOfStateinStateMachine(state) + 1; j < stateCells.length; j++)
                     {
                        State<E> checkState = stateMachine.states.get(j);

                        if (!doesCellAlreadyExist(checkState.getStateEnum().toString()))
                        {
                           stateCells[stateCellIndex] = createCell(checkState.getStateEnum().toString(), new Point2D(0, 0), stateCellIndex);
                           createArrow(stateCellIndex - 1, stateCellIndex);
                           stateCellIndex++;
                        }
                     }
                  }
               }
            }
         }
      }
   }

   private void createArrowsForStateTransitions()
   {
      for (int i = 0; i < stateCells.length; i++)
      {
         State<E> state = stateMachine.states.get(indexOfStateinStateMachine(stateCells[i].toString()));

         for (int z = 0; z < state.getStateTransitions().size(); z++)
         {
            int source = indexOfStateInStateCells(state);
            int target =
               indexOfStateInStateCells(stateMachine.states.get(indexOfStateinStateMachine(state.getStateTransitions().get(z).getNextStateEnum().toString())));
            createArrow(source, target);
         }
      }
   }

   private void createDefaultArrows()
   {
      for (int i = 0; i < stateMachine.states.size(); i++)
      {
         State<E> state = stateMachine.states.get(i);
         StateTransition<E> defaultNextStateTransition = state.getDefaultNextStateTransition();
         if (defaultNextStateTransition != null)
         {
            int source = indexOfStateInStateCells(state);
            int target = indexOfStateInStateCells(
                             stateMachine.states.get(
                                indexOfStateinStateMachine(defaultNextStateTransition.getNextStateEnum().toString())));
            createArrow(source, target);
         }
      }
   }

   private Color colorStateCell(State<E> state)
   {
      State<E> currentState = stateMachine.getCurrentState();

      if (state == currentState)
      {
         return Color.RED;
      }
      else
      {
         return Color.CYAN;
      }
   }

   private void initializeJPanel()
   {
      initiate = true;
      graph.setGraphLayoutCache(new GraphLayoutCache(new DefaultGraphModel(), new DefaultCellViewFactory(), true));
      graph.setEditable(false);

      for (int i = 0; i < numberOfStates; i++)
      {
         isStateEmpty(stateMachine.states.get(i));
      }

      stateCells = new DefaultGraphCell[numberOfStates];


      State<E> parent = getParentState();

      stateCells[stateCellIndex] = createCell(parent.getStateEnum().toString(), new Point2D(0, 0), stateCellIndex);
      stateCellIndex++;

      createStateMachineDiagram(parent);

      graph.getGraphLayoutCache().insert(stateCells);



      JGraphFacade facade = new JGraphFacade(graph.getGraphLayoutCache());
      facade.setEdgePromotion(false);
      JGraphTreeLayout layout = new JGraphTreeLayout();
      layout.run(facade);
      Map<?, ?> map = facade.createNestedMap(true, true);
      graph.getGraphLayoutCache().edit(map);
      createDefaultArrows();
      createArrowsForStateTransitions();
   }

   private Point2D[] stateCenters;
   private Vector2D tempVector2d = new Vector2D();

   private Dimension dimension = new Dimension();
   final static BasicStroke wideStroke = new BasicStroke(3.0f);

   private void OldStateMachineDiagram(Graphics g, double width, double height)
   {
      if (stateCenters == null)
      {
         stateCenters = new Point2D[numberOfStates];

         for (int i = 0; i < numberOfStates; i++)
         {
            stateCenters[i] = new Point2D();
         }
      }

      Graphics2D g2 = (Graphics2D) g;
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      g2.setStroke(wideStroke);

      // Determine the Ellipse Radii and Circumference:
      double Ry = 0.25 * height;
      double Rx = 0.25 * width;
      double circumference = approximateElipseCircumference(Rx, Ry);

      // Determine the Circle Radius:
      int circleRadiusBasedOnSpacing = ((int) (circumference / (2.5 * numberOfStates)));
      int circleRadiusBasedOnPanelSize = ((int) (0.9 * Math.min(Rx, Ry)));
      int circleRadius = Math.min(circleRadiusBasedOnSpacing, circleRadiusBasedOnPanelSize);

      // Draw a circle for each state:
      State<E> currentState = stateMachine.getCurrentState();

      for (int i = 0; i < numberOfStates; i++)
      {
         State<E> state = stateMachine.states.get(i);
         if (state == currentState)
            g.setColor(Color.RED);
         else
            g2.setColor(Color.BLACK);

         double angle = (2.0 * Math.PI * i) / (numberOfStates);
         stateCenters[i].setX(Rx * Math.cos(angle) + width / 2);
         stateCenters[i].setY(Ry * Math.sin(angle) + height / 2);

         // g2.drawOval(stateCentersX[i] - circleRadius, stateCentersY[i] - circleRadius, circleRadius*2, circleRadius*2);
         g2.draw(new Ellipse2D.Double(stateCenters[i].getX() - circleRadius, stateCenters[i].getY() - circleRadius, circleRadius * 2, circleRadius * 2));
         String stateString = state.getStateEnum().toString();

         g2.drawString(state.getStateEnum().toString(), (int) stateCenters[i].getX() - 8 * stateString.length() / 2, (int) stateCenters[i].getY());
      }

      g2.setColor(Color.BLACK);

      // Draw a line for each transition condition:
      for (int i = 0; i < numberOfStates; i++)
      {
         State<E> state = stateMachine.states.get(i);

         ArrayList<StateTransition<E>> stateTransitions = state.getStateTransitions();

         for (StateTransition<E> stateTransition : stateTransitions)
         {
            State<E> nextState = stateMachine.getState(stateTransition.getNextStateEnum());

            if (nextState == null)
            {
               System.err.println("Error. StateMachine doesn't include state with enum " + stateTransition.getNextStateEnum());

               return;
            }

            int nextIndex = stateMachine.states.indexOf(nextState);

            if (nextIndex >= 0)
            {
               tempVector2d.sub(stateCenters[nextIndex], stateCenters[i]);
               tempVector2d.normalize();
               tempVector2d.scale(circleRadius);

               g2.draw(new Line2D.Double(stateCenters[i].getX() + tempVector2d.getX(), stateCenters[i].getY() + tempVector2d.getY(), stateCenters[nextIndex].getX() - tempVector2d.getX(),
                                         stateCenters[nextIndex].getY() - tempVector2d.getY()));
            }
            else
            {
               System.err.println("Error. StateMachine doesn't include state " + nextState);
            }
         }
      }
   }

   private boolean initiate;
   private double tempHeight = 0;
   private double tempWidth = 0;

   private void reScalingStateMachine(double width, double height)
   {
      if (initiate)
      {
         tempWidth = width;
         tempHeight = height;
         initiate = false;
      }

      if (tempWidth > width)
      {
         graph.setScale(graph.getScale() + (width - tempWidth) / 3000);
         tempWidth = width;
      }

      if (tempWidth < width)
      {
         graph.setScale(graph.getScale() + (width - tempWidth) / 3000);
         tempWidth = width;
      }

      if (tempHeight > height)
      {
         graph.setScale(graph.getScale() - (tempHeight - height) / 3000);
         tempHeight = height;
      }

      if (tempHeight < height)
      {
         graph.setScale(graph.getScale() + (height - tempHeight) / 3000);
         tempHeight = height;
      }
   }

   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      this.getSize(dimension);
      double width = dimension.getWidth();
      double height = dimension.getHeight();

      if (oldStateDiagram)
      {
         OldStateMachineDiagram(g, width, height);
      }
      else
      {
         graph.setBounds(0, 0, (int) width, (int) height);
         graph.setAlignmentX(CENTER_ALIGNMENT);
         reScalingStateMachine(width, height);
         this.add(graph);
      }
   }

   private static double approximateElipseCircumference(double a, double b)
   {
      double x = (a - b) / (a + b);

      return Math.PI * (a + b) * (1.0 + 3.0 * x * x / (10.0 + Math.sqrt(4.0 - 3.0 * x * x)));
   }
}
