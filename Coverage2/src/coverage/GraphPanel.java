package coverage;

import java.awt.Image;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseListener;
import java.awt.Dimension;
import java.awt.Color;
import java.awt.event.MouseEvent;
import java.awt.Graphics;
import java.awt.FontMetrics;
import javax.swing.*;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Stroke;
import java.awt.Toolkit;
import java.awt.Graphics2D;
import java.awt.event.*;
import java.util.ArrayList;


 class GraphPanel extends JPanel implements Runnable, MouseListener, MouseMotionListener,KeyListener 
{
    Coverage coverage;
    Node nodes[];
//    Node base = new Node();
    ArrayList<Node> dataNodes = new ArrayList<Node>();
    boolean dataSourcePicked = false;
    
    Thread painter;

    Node pick;

    //group selection
    boolean InGroupSelection;
    int groupSelectionAnchorX, groupSelectionAnchorY, groupSelectionW, groupSelectionH;

    boolean pickfixed;
    Image offscreen;
    Dimension offscreensize;
    Graphics2D offgraphics;

    //constants for drawing
    static final int DISPLAY_MULTIPLIER = 12; //was 16
    static final int DRAWING_OFFSET = 20;
    //final Color nodeColor = Color.pink;
    final Color nodeColor = new Color(250, 220, 100);
    final Color selectColor = new Color(0, 191, 255);
    
    Stroke thickStroke = new BasicStroke (5.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND);
    Stroke thinStroke = new BasicStroke (1.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND);
    Stroke mediumStroke = new BasicStroke (3.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND);

    GraphPanel(Coverage cvrg)
    {
        //this.setPreferredSize(new Dimension(400,300));
        this.coverage = cvrg;
        addMouseListener(this);
        this.setFocusable(true);   // Allow this panel to get focus.
        this.addKeyListener(this); // listen to our own key events.

        
        nodes = new Node[coverage.MAX_ROBOT_NUMBER];
        for (int i2 = 0; i2 < coverage.MAX_ROBOT_NUMBER; i2++)
        {
            Node n = new Node(0,0,i2,Integer.toString(i2));
            nodes[i2] = n;
        }
        for(int i=0; i<coverage.density.MaxDataSourceNum ;i++)
        {
        	dataNodes.add(new Node(0,0,i,Integer.toString(i)));
        }
        read_nodes_pos();
    }

    public void run()
    {
        Thread me = Thread.currentThread();
        while (painter == me)
        {
            readyNextFrame(); //always sleep;
            //if (readyNextFrame()) //return value is whether the sleep is needed
            {
                try
                {
                    Thread.sleep(5);
                } catch (InterruptedException e)
                {
                    break;
                }
            }
        }
    }

    private synchronized boolean readyNextFrame()
    {
        long beginTime = System.nanoTime();
        if (coverage.start_demo)
        {
           
                try
                {
                    coverage.sim.simulateOneStep();

                    read_nodes_pos();
                    coverage.updateTable();
                } 
                catch (NullPointerException e)
                {
                	System.out.println(e.toString()+" "+e.getMessage());
                	for(int i =0; i<e.getStackTrace().length; i++)
                	{
                		System.out.println(e.getStackTrace()[i]);
                	}
                } 
                catch (IndexOutOfBoundsException e1)
                {
                	System.out.println(e1.toString()+" "+e1.getMessage());
                }
//                catch(Exception e2) //it's not advisable to catch Exception
//                {
//                	System.out.println(e2.toString()+" "+e2.getMessage());
//                }

            
        }

        if (coverage.InAggregationMode)
        {
            try
            {
                if (!coverage.aggregator.AggregationOneStep())
                {
                    coverage.aggregator.coreElementsUpdate();
                    coverage.InAggregationMode = false;
                }
            } catch (NullPointerException e)
            {
                coverage.InAggregationMode = false;
            }
        }
        //repaint();
        long passedTime = System.nanoTime() - beginTime; //if the above calculation took too long, there is no need to wait
        //System.out.println(passedTime);

        paintScreen();//the actual drawing of stuff
        return (passedTime < 5000000); //5ms??
    }

	public void read_nodes_pos() {
		for (int i1 = 0; i1 < coverage.MAX_ROBOT_NUMBER; i1++)
		{
		    nodes[i1].x = zoomIn(coverage.sim.robotList.get(i1).position.x);
		    nodes[i1].y = zoomIn(coverage.sim.robotList.get(i1).position.y);
		}
//		base.x = zoomIn(coverage.sim.robotList.get(0).position.x);
//		base.y = zoomIn(coverage.sim.robotList.get(0).position.y);	
		for (int i = 0; i < coverage.density.MaxDataSourceNum; i++)
		{
		    dataNodes.get(i).x = zoomIn(coverage.density.MovingDataSources.get(i).x);
		    dataNodes.get(i).y = zoomIn(coverage.density.MovingDataSources.get(i).y);
		}
	}
//	 protected void paintBase(Graphics g, FontMetrics fm)
//	    {
//	    	int w = 20;
//	    	int h = 20; 
//	        
//	        //paint the node itself
//	        int x = (int) coverage.sim.robotList.get(0).position.x;
//	        int y = (int) coverage.sim.robotList.get(0).position.y;;
//	    	
//	        g.setColor(Color.pink);
//	        //  g.setFont(new Font("Sans-serif", Font.BOLD, 18));   
//	        g.fillOval(x - w / 2, y - h / 2, w, h);
//	        g.setColor(Color.black);
//	        g.drawOval(x - w / 2 , y - h / 2 , w, h);
//	        g.drawString("L", x-fm.stringWidth("L")/2, y+4);
//
//	    }
	
    protected void paintNode(Graphics g, Node n, FontMetrics fm)
    {

        //int w = fm.stringWidth(n.lbl) + 15; //originally it is 4, if you change this number, do not forget to change the drawstring call a few lines below
        //int h = fm.getHeight() + 10; //originally it is 0
    	int w = 20;
    	int h = 20; 
        

        //draw estimated position
     if(!coverage.scanMode)
     {
         g.setColor(Color.black);
         int estimatedX = (int) zoomIn(coverage.sim.robotList.get(n.id).estimatedPosition.x);
         int estimatedY = (int) zoomIn(coverage.sim.robotList.get(n.id).estimatedPosition.y);
      //   if(coverage.sim.robotList.get(n.id).showRed)
    //     {
       //      g.setColor(Color.red);
     //    }
         g.drawOval(estimatedX - w / 2, estimatedY - h / 2, w , h );
         g.drawString(n.lbl, estimatedX - fm.stringWidth(n.lbl)/2, estimatedY +4);
     }

        //paint the node itself
        int x = (int) n.x;
        int y = (int) n.y;
        g.setColor((n.selected) ? selectColor : nodeColor);
        //  g.setFont(new Font("Sans-serif", Font.BOLD, 18));   
        g.fillOval(x - w / 2, y - h / 2, w, h);
        g.setColor(Color.black);
        g.drawOval(x - w / 2, y - h / 2, w, h);
        if(n.id==0)
        {
	        g.drawString("L", x-fm.stringWidth("L")/2, y+4);
        }
        else
        {
        	g.drawString(n.lbl, x-fm.stringWidth(n.lbl)/2, y+4);
        }

        g.setColor(Color.black);
        int DIRECTION_POINTER_SIZE = 6;
        g.fillOval(x+(int)(Math.cos(coverage.sim.robotList.get(n.id).heading)*w/2)-DIRECTION_POINTER_SIZE/2, 
        		y+(int)(Math.sin(coverage.sim.robotList.get(n.id).heading)*h/2)-DIRECTION_POINTER_SIZE/2, 
        		DIRECTION_POINTER_SIZE,DIRECTION_POINTER_SIZE);
        
        //comment out the following three lines to disable showing where Fov edge hits
//        g.setColor(Color.magenta);
//        g.drawLine(x,y,(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_impact_point_left.x),(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_impact_point_left.y));
//        g.drawLine(x,y,(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_impact_point_right.x),(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_impact_point_right.y));       
        
        if(coverage.sim.robotList.get(n.id).FOV_degree<Math.PI*2)
        {
	        g.setColor(Color.blue);
	        g.drawLine(x,y,(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_point_left.x),(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_point_left.y));
	        g.drawLine(x,y,(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_point_right.x),(int) zoomIn(coverage.sim.robotList.get(n.id).FOV_edge_point_right.y));       
        }
      //draw the sensing radius
      //drawOval seems to be a performance hit
        //draw the sensing radius
//        g.setColor(Color.gray);
//        int cutoffRange = 15;
//        if (coverage.simulationMode)
//        {
//            cutoffRange = (int) coverage.sim.robotList.get(n.id).sensingCutoffRange;
//        }
        //drawOval seems to be a performance hit
        //g.drawOval(x - cutoffRange * DISPLAY_MULTIPLIER, y - cutoffRange * DISPLAY_MULTIPLIER, 2 * cutoffRange * DISPLAY_MULTIPLIER,
             //      2 * cutoffRange * DISPLAY_MULTIPLIER);
        //     g.setFont(new Font("Sans-serif", Font.PLAIN, 12));

    }

    //  public synchronized void update(Graphics g)
    //   public synchronized void paintComponent(Graphics g)
    protected synchronized void paintScreen()
    {
        //super.paintComponent(g);

        Dimension d = getSize();
        
        for(int i=0; i<11; i++)
        {
        	//System.out.println("d.width: "+d.width+" d.height: "+d.height);
        	if(d.width <= 0 || d.height <=0)
        	{
        		d = getSize();
        		try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
        	}
        	else
        	{
        		break;
        	}
        }
        
        if ((offscreen == null) || (d.width != offscreensize.width) || (d.height != offscreensize.height))
        {
            offscreen = createImage(d.width, d.height);
            offscreensize = d;
            if (offgraphics != null)
            {
                offgraphics.dispose();
            }
            //offgraphics = offscreen.getGraphics(); //not such a big difference with Graphics2D
            offgraphics = (Graphics2D) offscreen.getGraphics();
            offgraphics.setFont(getFont());
        }

        //redraw the background of the whole graph area
        //offgraphics.setColor(getBackground());
        offgraphics.setColor(Color.white);
        offgraphics.fillRect(0, 0, d.width, d.height);

        FontMetrics fm = offgraphics.getFontMetrics();

        //draw tick
        offgraphics.setColor(Color.black);
        for (int i4 = 0; i4 <= coverage.world_size_x; i4 = i4 + 5)
        {
            offgraphics.drawString(Integer.toString(i4), (int) zoomIn(i4) - 4, DRAWING_OFFSET - fm.getHeight() + 12);
        }
        for (int i4 = 0; i4 <= coverage.world_size_y; i4 = i4 + 5)
        {
            offgraphics.drawString(Integer.toString(i4), DRAWING_OFFSET - fm.stringWidth(Integer.toString(i4)) - 3, (int) zoomIn(i4) + 4);
        }

        //draw coverage map
        if (this.coverage.evaluateObjFunc)
        {
            double inc = coverage.sim.objEvalIncrement;
            float prob;
            int interval = (int) (inc * DISPLAY_MULTIPLIER);
            try
            {
                for (int i = 0; i < this.coverage.sim.coverageMap.length; i++)
                {
                    for (int j = 0; j < coverage.sim.coverageMap[i].length; j++)
                    {
                        prob = (float) coverage.sim.coverageMap[i][j];

                        if (prob > 0.5)
                        {
                            offgraphics.setColor(new Color(1f, 1f, 2f * (prob - 0.5f)));
                        }
                        else if (prob > 0.03)
                        {
                            offgraphics.setColor(new Color(0f, 1f, prob * 2));
                        }
                        else
                        {
                            //offgraphics.setColor(new Color(50,150,50));
                            offgraphics.setColor(new Color(0,255,0));
                        }
                        offgraphics.fillRect((int) zoomIn(j * inc), (int) zoomIn(i * inc), interval, interval);
                    }
                }
            } catch (ArrayIndexOutOfBoundsException e)
            {
            }
        }
        
        //draw Event density
        if(coverage.ShowEventDensity)
        {
	        double eventDensityIncrement = 0.5;
	        int interval1 = (int) (eventDensityIncrement * DISPLAY_MULTIPLIER);
	        //offgraphics.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.5f));
	        for(double i = eventDensityIncrement/2; i < coverage.world_size_x; i += eventDensityIncrement)
	        {
	        	for(double j = eventDensityIncrement/2; j < coverage.world_size_y; j += eventDensityIncrement)
	        	{
	        		double densityColor = coverage.density.GetEventDensity(new point2(i,j), coverage.sim.simulationTime);
	        		int colorIntensity = Math.min(255,(int)(densityColor*255/100));
	        		//if(densityColor>6600) {System.out.println("densityColor@ "+i+","+j+" is "+densityColor);}
	        		offgraphics.setColor(new Color(255,255-colorIntensity/2,255-colorIntensity));
	        		 
	        		offgraphics.fillRect((int) zoomIn(i -eventDensityIncrement/2), (int) zoomIn(j-eventDensityIncrement/2), interval1, interval1);
	        	}
	        }
	        //offgraphics.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 1.0f));
	        double gridIncrement = 2.0;
	        offgraphics.setColor(Color.gray);
	        for(double i = gridIncrement; i < coverage.world_size_x; i += gridIncrement)
	        {
	        	offgraphics.drawLine((int) zoomIn(i), (int) zoomIn(0), (int) zoomIn(i), (int) zoomIn(coverage.world_size_y));
	        }
	        for(double j = gridIncrement; j < coverage.world_size_y; j += gridIncrement)
        	{
        		offgraphics.drawLine((int) zoomIn(0), (int) zoomIn(j), (int) zoomIn(coverage.world_size_x), (int) zoomIn(j));
        	}
        }
        
        //draw scan map
        if(coverage.scanMode)
        {
            double inc = coverage.sim.scanIncrement;
            int interval = (int) (inc * DISPLAY_MULTIPLIER);
            try
            {
                for (int i = 0; i < this.coverage.sim.scanMap.length; i++)
                {
                    for (int j = 0; j < coverage.sim.scanMap[i].length; j++)
                    {
                        if (coverage.sim.scanMap[i][j] > coverage.sim.scanValueBase)
                        {
                            offgraphics.setColor(scanColorChooser((coverage.sim.scanMap[i][j] - coverage.sim.scanValueBase) /
                                    coverage.sim.scanValueDiff));
                        }
                        else
                        {
                            offgraphics.setColor(Color.GREEN);
                        }
                        offgraphics.fillRect((int) zoomIn(j * inc), (int) zoomIn(i * inc), interval, interval);
                    }
                }
             } catch (ArrayIndexOutOfBoundsException e)
            {
            }

        }

        //draw boundary
        offgraphics.setColor(Color.black);
        offgraphics.drawRect(DRAWING_OFFSET, DRAWING_OFFSET, coverage.world_size_x * DISPLAY_MULTIPLIER,
                             coverage.world_size_y * DISPLAY_MULTIPLIER);
        //draw obstacles
        offgraphics.setColor(Color.blue);
        try
        {
            for (int i3 = 0; i3 < coverage.obstacle_number; i3++)
            {
            	this.offgraphics.fillPolygon(coverage.obstaclesForDrawing[i3]);          	
            }
        } catch (NullPointerException e)
        {
        	e.printStackTrace();
        }

        //draw moving target, should be commented out most of the time
        /*offgraphics.setColor(Color.red);
               if(graph.current_frame>500 && graph.current_frame<1500)
               {
         offgraphics.fillOval((graph.current_frame-500)*60*graph.display_multiplier/1000-7*graph.display_multiplier+graph.drawing_offset,
         (graph.current_frame-500)*50*graph.display_multiplier/1000-7*graph.display_multiplier+graph.drawing_offset,
                                        14*graph.display_multiplier,14*graph.display_multiplier);
                 }*/

        //draw moving targets (Gaussian peaks of event density)
        {
        	if(coverage.sim.target1.trajectoryStarted&& !coverage.sim.target1.trajectoryEnded)
        	{
        		offgraphics.setColor(new Color(178,0,255));
        		offgraphics.fillOval((int) zoomIn(coverage.sim.target1.x) - 20,
                        (int) zoomIn(coverage.sim.target1.y) - 20, 40, 40);
        	}
        }
        
        
        //draw robots
        //if (this.coverage.data_loaded)
        {

            //paint trajectory
                if (coverage.showTrajectory)
                {
                    for (int i8 = 0; i8 < coverage.sim.realTimeRobotNumber; i8++)
                    {
                        int index = coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryIndex;
                        if (index > 1)
                        {
                            offgraphics.setColor(Color.black);
                            for (int i = 1; i < index; i++)
                            {
                                offgraphics.drawLine((int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryX[i]),
                                        (int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryY[i]),
                                        (int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryX[i + 1]),
                                        (int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryY[i + 1]));
                            }
                        }
                    }
                }
      
            //draw communication link
            if(this.coverage.is_link_cost_mode)
            {
	            offgraphics.setColor(Color.BLACK);
	            offgraphics.setStroke(new BasicStroke(3));
	            for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
	            {
	            	if(coverage.sim.robotList.get(i).predecessor != null)
	            	{
	            		if(coverage.sim.robotList.get(i).is_upstream_connectivity_in_danger)
	            		{
	            			offgraphics.setColor(Color.RED);
	            		}	            			            		
	            		
		            	offgraphics.drawLine((int) zoomIn(coverage.sim.robotList.get(i).position.x),
		            			(int) zoomIn(coverage.sim.robotList.get(i).position.y),
		            			(int) zoomIn(coverage.sim.robotList.get(i).predecessor.position.x),
		            			(int) zoomIn(coverage.sim.robotList.get(i).predecessor.position.y));
		            	
//		            	if(coverage.sim.robotList.get(i).is_stretching_upstream)
//	            		{
//	            			offgraphics.setColor(Color.BLACK);
//	            			offgraphics.drawLine((int) zoomIn((coverage.sim.robotList.get(i).position.x+coverage.sim.robotList.get(i).predecessor.position.x)/2),
//			            			(int) zoomIn((coverage.sim.robotList.get(i).position.y+coverage.sim.robotList.get(i).predecessor.position.y)/2),
//			            			(int) zoomIn(coverage.sim.robotList.get(i).predecessor.position.x),
//			            			(int) zoomIn(coverage.sim.robotList.get(i).predecessor.position.y));
//	            		}	
		            	
		            	offgraphics.setColor(Color.BLACK);
		            	
		            	//TODO show the communication cost to base next to node, hide for now
//		            	String node_cost_to_base_str = Double.toString( coverage.sim.robotList.get(i).costToBase);
//		            	if(node_cost_to_base_str.length()>=7) //TODO only show 7 digits, tunable
//		            	{
//		            		node_cost_to_base_str = node_cost_to_base_str.substring(0,7); 	
//		            	}
		            	
//		            	offgraphics.drawString(node_cost_to_base_str,
//		            			(int) zoomIn((coverage.sim.robotList.get(i).position.x)),
//		            			(int) zoomIn((coverage.sim.robotList.get(i).position.y - 0.6)));
	            	}
	            }
	            offgraphics.setStroke(new BasicStroke(1));
            }
            
            //just draw the number of nodes user want
            for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
            {
                paintNode(offgraphics, nodes[i], fm);

            }
            
//            //draw base station
//            if(coverage.is_link_cost_mode || coverage.has_connectivity_constraint)
//            {
//            	paintBase(offgraphics,fm);
//            }

        }

        //draw active events
       offgraphics.setColor(Color.red);  //undetect events should be draw with another color
       for(int i = 0; i<coverage.sim.currentDetectedEvents.size(); i++)
       {
           Event currentEvent = coverage.sim.currentDetectedEvents.get(i);
           drawStarShape(currentEvent.eventLocation);
           offgraphics.drawString(Integer.toString((int)(100.0*currentEvent.monitorPercentage)), (int)zoomIn(currentEvent.eventLocation.x) - 6, (int)zoomIn(currentEvent.eventLocation.y) - 6 );
       }
      

       //draw moving data source
       if(coverage.has_moving_data_source)
       {
    	   offgraphics.setColor(Color.red);  
    	   offgraphics.setStroke(mediumStroke);
    	   for(int i = 0; i<coverage.density.ActiveDataSourceNum;i++)
    	   {
    		   drawStarShape(coverage.density.MovingDataSources.get(i));
    	   }
    	   offgraphics.setStroke(thinStroke);
       }

        //draw elements in Aggregation mode
        if (coverage.ShowAggregation)
        {
            offgraphics.setColor(Color.BLACK);
            if (coverage.InAggregationMode)
            {
                for (int i = 0; i < coverage.aggregator.currentElements.size(); i++)
                {
                    offgraphics.drawOval((int) zoomIn(coverage.aggregator.currentElements.get(i).position.x) - 2,
                                         (int) zoomIn(coverage.aggregator.currentElements.get(i).position.y) - 2, 4, 4);
                }
            }
            else //just show cores
            {
                for (int i = 0; i < coverage.aggregator.coreElements.size(); i++)
                {
                    offgraphics.drawString(Double.toString(coverage.aggregator.coreElements.get(i).weight),
                                           (int) zoomIn(coverage.aggregator.coreElements.get(i).position.x),
                                           (int) zoomIn(coverage.aggregator.coreElements.get(i).position.y));
                }

            }
        }

        //draw group selection rectangle
        if (InGroupSelection)
        {
            offgraphics.setColor(Color.gray);
            this.offgraphics.drawRect(groupSelectionAnchorX, groupSelectionAnchorY, groupSelectionW, groupSelectionH);
        }

        Graphics g;
        try
        {
            g = this.getGraphics();
            if (g != null)
            {
                g.drawImage(offscreen, 0, 0, null);
            }
            Toolkit.getDefaultToolkit().sync(); // sync the display on some systems
            g.dispose();
        } catch (Exception e) // quite commonly seen at applet destruction
        {
            System.out.println("Graphics Context error: " + e);
            System.out.println(e.toString());
        }

        //g.drawImage(offscreen, 0, 0, null);
    }

    //1.1 event handling
    public void mouseClicked(MouseEvent e)
    {
        if(e.isShiftDown()){
            //assuming events added manually are all detected for now
            Event newEvent = new Event(zoomOut(e.getX()),zoomOut(e.getY()),coverage.sim.simulationTime);
            this.coverage.sim.currentDetectedEvents.add(newEvent);
            coverage.sim.updateEventCollectionPercentage();
        }

        if(e.isControlDown())
        {
            for (int i = 0; i < coverage.sim.currentDetectedEvents.size(); i++)
            {
                if (Math.abs(e.getX() - zoomIn(coverage.sim.currentDetectedEvents.get(i).eventLocation.x)) < 8
                    && Math.abs(e.getY() - zoomIn(coverage.sim.currentDetectedEvents.get(i).eventLocation.y)) < 8)
                {
                    coverage.sim.currentDetectedEvents.remove(i);
                    break;
                }
            }
        }
    }

    public void mousePressed(MouseEvent e)
    {
     
            if (e.getButton() == MouseEvent.BUTTON1)
            {
                addMouseMotionListener(this);

                //double bestdist = Double.MAX_VALUE; //enable indirect select
                double bestdist = 144; //at least within 12 pixels, not 144 because dist is squared distance
                int x = e.getX();
                int y = e.getY();
                double dist;
                
//                if(this.coverage.is_link_cost_mode || this.coverage.has_connectivity_constraint)
//                {
//	                dist = (base.x - x) * (base.x - x) + (base.y - y) * (base.y - y);
//	                if (dist < bestdist)
//	                {
//	                    pick = base;
//	                    bestdist = dist;
//	                }
//                }
                
                for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
                {
                    Node n = nodes[i];
                    dist = (n.x - x) * (n.x - x) + (n.y - y) * (n.y - y);
                    if (dist < bestdist)
                    {
                        pick = n;
                        bestdist = dist;
                    }
                }
                
                for (int i = 0; i < coverage.density.ActiveDataSourceNum; i++)
                {
                    Node n = dataNodes.get(i);
                    dist = (n.x - x) * (n.x - x) + (n.y - y) * (n.y - y);
                    if (dist < bestdist)
                    {   
                    	dataSourcePicked = true;
                        pick = n;
                        bestdist = dist;
                    }
                }
                

                if (pick != null)
                {
                    //pickfixed = pick.fixed;
                    pick.selected = true;
                    // pick.x = x;
                    // pick.y = y;
                }
                else
                {
                    this.groupSelectionAnchorX = e.getX();
                    this.groupSelectionAnchorY = e.getY();
                }
                paintScreen();
                //repaint();
                e.consume();
            }
        
    }

    public void mouseReleased(MouseEvent e)
    {
        //System.out.println("mouseReleased"+e.toString());
        if (e.getButton() == MouseEvent.BUTTON1)
        {
            removeMouseMotionListener(this);
            if (pick != null)
            {
                //pick.x = e.getX();
                //pick.y = e.getY();
                pick.selected = false;
                pick = null;
                dataSourcePicked = false;

                if (coverage.evaluateObjFunc)
                {
                    coverage.sim.UpdateObjInfo();
                }

                coverage.sim.updateEventCollectionPercentage();
                if(coverage.is_link_cost_mode || coverage.has_connectivity_constraint)
                {
                	coverage.sim.pathGenerator.buildRoutingTree();
                }
            }
            else
            {
                //group selection behavior
                for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
                {
                    if (nodes[i].x > this.groupSelectionAnchorX && nodes[i].x < groupSelectionAnchorX + this.groupSelectionW &&
                        nodes[i].y > this.groupSelectionAnchorY && nodes[i].y < groupSelectionAnchorY + this.groupSelectionH)
                    {
                        nodes[i].selected = true;
                    }
                    else
                    {
                        nodes[i].selected = false;
                    }

                }
                groupSelectionW = 0;
                groupSelectionH = 0;
                InGroupSelection = false;
            }
            paintScreen();
            //repaint();
            e.consume();         
        }

        if (e.getButton() == MouseEvent.BUTTON2)
        {
            coverage.pause.doClick();
        }

        if (e.getButton() == MouseEvent.BUTTON3)
        { //System.out.println("mouseReleased"+e.toString());
            for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
            {
                if (nodes[i].selected)
                {
                    nodes[i].x = e.getX();
                    nodes[i].y = e.getY();
                    coverage.sim.robotList.get(i).position.x = zoomOut(e.getX());
                    coverage.sim.robotList.get(i).position.y = zoomOut(e.getY());
                    coverage.sim.robotList.get(i).set_FOV_edge_points();
                    //heading reset is optional
                    coverage.sim.robotList.get(i).heading = 0;
                }
            }
            coverage.updateTable();
            if (coverage.evaluateObjFunc)
            {
                 coverage.sim.UpdateObjInfo();
            }
            coverage.sim.updateEventCollectionPercentage();
            if(this.coverage.is_link_cost_mode || this.coverage.has_connectivity_constraint)
            {
            	coverage.sim.pathGenerator.buildRoutingTree();
            }
        }

        this.requestFocus();
    }

    public void mouseEntered(MouseEvent e)
    {
    }

    public void mouseExited(MouseEvent e)
    {
    }

    public void mouseDragged(MouseEvent e)
    {
        //System.out.println("mouseDragged"+e.toString());
        // System.out.println("mouseDragged"+e.getModifiers());
        if (e.getModifiers() == 16) //16 is left mouse button
        {
            if (pick != null)
            {
                pick.x = e.getX();
                pick.y = e.getY();

//                if(base.selected)
//                {
//                	 coverage.sim.baseRobot.position.x = zoomOut(e.getX());
//                     coverage.sim.baseRobot.position.y = zoomOut(e.getY());
//                     coverage.sim.baseRobot.estimatedPosition.x = zoomOut(e.getX());
//                     coverage.sim.baseRobot.estimatedPosition.y = zoomOut(e.getY());
//                }
//                else if(dataSourcePicked)
                if(dataSourcePicked)
                {
                	double newHeading = Math.atan2(zoomOut(e.getY())-coverage.density.MovingDataSources.get(pick.id).y, zoomOut(e.getX())-coverage.density.MovingDataSources.get(pick.id).x);
                	coverage.density.MovingDataSources.get(pick.id).x = zoomOut(e.getX());
                	coverage.density.MovingDataSources.get(pick.id).y = zoomOut(e.getY());
                	coverage.density.MovingDataSources.get(pick.id).Heading = newHeading;
                }
                else
                {
	                coverage.sim.robotList.get(pick.id).position.x = zoomOut(e.getX());
	                coverage.sim.robotList.get(pick.id).position.y = zoomOut(e.getY()); 
	                coverage.sim.robotList.get(pick.id).set_FOV_edge_points();
	                
	                //coverage.RobotTable.setValueAt((e.getX() - coverage.drawing_offset) / (double) coverage.display_multiplier, pick.id, 1);
	                //coverage.RobotTable.setValueAt((e.getY() - coverage.drawing_offset) / (double) coverage.display_multiplier, pick.id, 2);
	                coverage.updateTable();
                }
            }
            else
            {
                InGroupSelection = true;
                this.groupSelectionW = e.getX() - this.groupSelectionAnchorX;
                this.groupSelectionH = e.getY() - this.groupSelectionAnchorY;
            }

        }
        /* if(e.getModifiers()==4)//4 is right mouse button
         {

         }
         if(e.getModifiers()==8)//8 is middle mouse button
         {
              coverage.start_demo = !coverage.start_demo;
         }
         */
        paintScreen();
        //repaint();
        e.consume();

    }

    public void mouseMoved(MouseEvent e)
    {
    }

    public void start()
    {
        painter = new Thread(this);
        painter.start();
    }

    public void stop()
    {
        painter = null;
    }

    public static double zoomIn(double smallValue)
    {
        return smallValue * DISPLAY_MULTIPLIER + DRAWING_OFFSET;
    }

    public static double zoomOut(double bigValue)
    {
        return (bigValue - DRAWING_OFFSET) / DISPLAY_MULTIPLIER;
    }

    protected Color scanColorChooser(double colorValue)
    {
        float colorValueFloat = (float)colorValue;
        if(colorValue<0||colorValue>1)
        {
            System.out.println("error choosing color:"+colorValue+" "+coverage.sim.scanValueBase+" "+coverage.sim.scanValueDiff);
             return Color.green;
        }
        else if(colorValue<=0.25)
        {
            colorValueFloat=colorValueFloat/0.25f;
            return new Color(0f,colorValueFloat ,1f);
        }
        else if (colorValue<=0.5)
        {
            colorValueFloat=(colorValueFloat-0.25f)/0.25f;
            return new Color(colorValueFloat,1f, 1f-colorValueFloat);
        }
        else if (colorValue<=0.75)
               {
                   colorValueFloat=(colorValueFloat-0.5f)/0.25f;
                   return new Color(1f,1f-colorValueFloat, 0f);
        }
        else if (colorValue<=1)
               {
                   colorValueFloat=(colorValueFloat-0.75f)/0.25f;
                   return new Color(1f-colorValueFloat,0f, 0f);
        }
        else
        {
            System.out.println("error choosing color:"+colorValue);
             return Color.green;
        }
    }

    //draw an event symbol
    protected void drawStarShape(point2 p)
    {
        double starLength = 0.8; //was 0.5
        double pi = 3.1415926;

        double x = p.x-starLength/3.0;  //make the center of the star the event location. 3.0 is empirical. I am too lazy to find the exact value.
        double y = p.y+starLength/3.0;
        double angle = pi;
        double xNext;
        double yNext;

        for(int i = 0; i<5; i++)
        {
            xNext = x+starLength*Math.cos(angle);
            yNext = y+starLength*Math.sin(angle);
            offgraphics.drawLine((int) zoomIn(x),
                                        (int) zoomIn(y),
                                        (int) zoomIn(xNext),
                                        (int) zoomIn(yNext));
            x = xNext;
            y = yNext;
            angle = angle + 0.8*pi;
            xNext = x+starLength*Math.cos(angle);
            yNext = y+starLength*Math.sin(angle);
           offgraphics.drawLine((int) zoomIn(x),
                                       (int) zoomIn(y),
                                       (int) zoomIn(xNext),
                                       (int) zoomIn(yNext));
           x = xNext;
           y = yNext;
           angle = angle - 0.4*pi;
        }

        //draw the event location
       // offgraphics.drawLine((int) zoomIn(p.x),
       //                            (int) zoomIn(p.y),
        //                           (int) zoomIn(p.x),
       //                            (int) zoomIn(p.y));

    }
    
    public void keyTyped(KeyEvent e) {	
    	boolean need_redraw = false;
    	//displayInfo(e, "KeyTyped");
    	if (e.getKeyChar()=='a')
    	{
    	 for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
         {
             if (nodes[i].selected)
             {     
            	 need_redraw = true;
                 coverage.sim.robotList.get(i).modify_sensor_heading(-0.05);
                 
             }
         }
    	}
    	else if(e.getKeyChar()=='d')
    	{
       	 for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
            {
                if (nodes[i].selected)
                {     
                	need_redraw = true;
                    coverage.sim.robotList.get(i).modify_sensor_heading(0.05);                  
                }
            }
       	}
    	
    		if (need_redraw && coverage.evaluateObjFunc)
            {
                 coverage.sim.UpdateObjInfo();     
            }
    }
    
    public void keyPressed(KeyEvent e) {
    	boolean need_redraw = false;
    	if (e.getKeyCode() == KeyEvent.VK_LEFT)
    	{
    	 for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
         {
             if (nodes[i].selected)
             {     
            	 need_redraw = true;
                 coverage.sim.robotList.get(i).modify_sensor_heading(-0.05);
                 
             }
         }
    	}
    	else if(e.getKeyCode() == KeyEvent.VK_RIGHT)
    	{
       	 for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
            {
                if (nodes[i].selected)
                {     
                	need_redraw = true;
                    coverage.sim.robotList.get(i).modify_sensor_heading(0.05);                  
                }
            }
       	}
    	
    	if (need_redraw && coverage.evaluateObjFunc)
        {
            coverage.sim.UpdateObjInfo();      
        }
    }
    
    public void keyReleased(KeyEvent e) {
    };
    
 private void displayInfo(KeyEvent e, String keyStatus){
        
        //You should only rely on the key char if the event
        //is a key typed event.
        int id = e.getID();
        String keyString;
        if (id == KeyEvent.KEY_TYPED) {
            char c = e.getKeyChar();
            keyString = "key character = '" + c + "'";
        } else {
            int keyCode = e.getKeyCode();
            keyString = "key code = " + keyCode
                    + " ("
                    + KeyEvent.getKeyText(keyCode)
                    + ")";
        }
        
        int modifiersEx = e.getModifiersEx();
        String modString = "extended modifiers = " + modifiersEx;
        String tmpString = KeyEvent.getModifiersExText(modifiersEx);
        if (tmpString.length() > 0) {
            modString += " (" + tmpString + ")";
        } else {
            modString += " (no extended modifiers)";
        }
        
        String actionString = "action key? ";
        if (e.isActionKey()) {
            actionString += "YES";
        } else {
            actionString += "NO";
        }
        
        String locationString = "key location: ";
        int location = e.getKeyLocation();
        if (location == KeyEvent.KEY_LOCATION_STANDARD) {
            locationString += "standard";
        } else if (location == KeyEvent.KEY_LOCATION_LEFT) {
            locationString += "left";
        } else if (location == KeyEvent.KEY_LOCATION_RIGHT) {
            locationString += "right";
        } else if (location == KeyEvent.KEY_LOCATION_NUMPAD) {
            locationString += "numpad";
        } else { // (location == KeyEvent.KEY_LOCATION_UNKNOWN)
            locationString += "unknown";
        }
        
        String newline = System.getProperty("line.separator");

        System.out.println(keyStatus + newline
        + "    " + keyString + newline
        + "    " + modString + newline
        + "    " + actionString + newline
        + "    " + locationString + newline);

    }


}
